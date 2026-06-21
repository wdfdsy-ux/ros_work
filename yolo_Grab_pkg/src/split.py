#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLOv8-seg 视觉抓取分割节点

订阅:
  /hand_camera/image_raw   — 相机图像
  /hand_camera/camera_info — 相机内参

发布:
  TF: hand_cam_link → object_{class}_{id}   (通过 DynamicTFPublisher)

流程:
  图像 → YOLOv8-seg → 掩码轮廓 → minAreaRect角点 → PnP → 发布TF
"""

import cv2
import numpy as np
import rospy
import sys
sys.path.append('/home/reicom2025/reinovo_visual_tutorial')
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from reirobot_API import DynamicTFPublisher
import tf.transformations as tft
import os
from std_msgs.msg import String

_done = False  # 全局标志: 识别一次后停止

# 物块类别
CLASS_NAMES = {
    0: 'chips',
    1: 'lens',
    2: 'camera',
    3: 'pcb',
    4: 'rotor',
    5: 'servo',
}

# 默认物块3D尺寸 (宽, 高, 厚度) 单位: 米
DEFAULT_DIMS = {
    'chips':  (0.050, 0.050, 0.010),
    'lens':   (0.030, 0.030, 0.015),
    'camera': (0.080, 0.050, 0.030),
    'pcb':    (0.100, 0.070, 0.015),
    'rotor':  (0.040, 0.040, 0.020),
    'servo':  (0.055, 0.040, 0.025),
}


def order_corners(pts):
    """将4个角点排序: 左上→右上→右下→左下"""
    rect = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    d = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(d)]
    rect[3] = pts[np.argmax(d)]
    return rect


def get_3d_corners(class_name, dims):
    """返回物块顶面4个角点在物体坐标系下的坐标 (米)"""
    w, h, _ = dims.get(class_name, (0.05, 0.05, 0.02))
    return np.array([
        [-w/2, -h/2, 0.0],
        [ w/2, -h/2, 0.0],
        [ w/2,  h/2, 0.0],
        [-w/2,  h/2, 0.0],
    ], dtype=np.float64)


def camera_info_cb(msg, cam_matrix, dist_coeffs):
    """相机内参回调"""
    if cam_matrix[0][0] == 0:
        cam_matrix[:] = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        dist_coeffs[:] = np.array(msg.D, dtype=np.float64)
        rospy.loginfo("相机内参已获取")


def image_cb(msg, bridge, model, rei_tf, cam_matrix, dist_coeffs, dims, det_pub):
    """图像回调 — 核心推理 + PnP + TF 发布 (只执行一次)"""
    global _done
    if _done:
        return

    # 等待内参就绪
    if cam_matrix[0][0] == 0:
        return

    # 图像转换
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    except CvBridgeError as e:
        rospy.logerr(f"图像转换失败: {e}")
        return

    # YOLOv8-seg 推理
    results = model(cv_img, conf=0.5, iou=0.45, verbose=False)
    result = results[0]

    # 清空上一帧的 TF
    rei_tf.clear_all_tf()

    obj_count = 0
    det_lines = []  # 收集检测信息行 (供 Grab_2.cpp)

    if result.masks is not None and result.boxes is not None:
        for i in range(len(result.boxes)):
            cls_id = int(result.boxes.cls[i])
            conf = float(result.boxes.conf[i])

            if conf < 0.5:
                continue

            class_name = CLASS_NAMES.get(cls_id, f'unknown_{cls_id}')

            # 提取掩码多边形 → minAreaRect 角点
            polygon = result.masks.xy[i]
            if len(polygon) < 4:
                continue

            pts = np.array(polygon, dtype=np.float32)
            rect = cv2.minAreaRect(pts)
            _, (bw, bh), _ = rect
            if bw < 5 or bh < 5:
                continue

            corners_2d = order_corners(cv2.boxPoints(rect))

            # PnP 解算
            corners_3d = get_3d_corners(class_name, dims)

            try:
                success, rvec, tvec = cv2.solvePnP(
                    corners_3d, corners_2d,
                    cam_matrix, dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )
                if not success:
                    continue
            except cv2.error as e:
                rospy.logwarn_throttle(10, f"solvePnP failed: {e}")
                continue

            # 旋转向量 → 四元数
            R_mat, _ = cv2.Rodrigues(rvec)
            rot_4x4 = np.eye(4)
            rot_4x4[:3, :3] = R_mat
            quat = tft.quaternion_from_matrix(rot_4x4)

            tx, ty, tz = float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])
            qx, qy, qz, qw = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])

            # 通过 DynamicTFPublisher 发布 TF
            tf_name = f'object_{class_name}_{i}'
            rei_tf.add_dynamic_tf('hand_cam_link', tf_name, [tx, ty, tz], [qx, qy, qz, qw])

            rospy.loginfo(f"[{i}] {class_name:8s} conf={conf:.2f}  "
                          f"pos=({tx:.3f},{ty:.3f},{tz:.3f})  "
                          f"tf={tf_name}")

            det_lines.append(f'{i}|{class_name}|{conf:.2f}|{tx:.4f},{ty:.4f},{tz:.4f}|{qx:.4f},{qy:.4f},{qz:.4f},{qw:.4f}')
            obj_count += 1

    rospy.loginfo(f"当前帧检测到 {obj_count} 个物块")

    # 发布一次后退出
    if obj_count > 0 and det_lines:
        det_pub.publish(String('\n'.join(det_lines)))
        _done = True
        rospy.sleep(0.3)  # 等待后台线程完成 TF 广播 + 话题投递
        rospy.loginfo("TF 与检测信息已发布，节点退出")
        rospy.signal_shutdown("检测完成，单次发布退出")


def main():
    rospy.init_node('yolo_seg_grasp', anonymous=False)

    # 模型路径
    model_path = rospy.get_param('~model_path',
                                  os.path.join(os.path.dirname(__file__), '..', 'best.pt'))

    # 加载模型
    rospy.loginfo(f"加载模型: {model_path}")
    model = YOLO(model_path)
    rospy.loginfo("模型加载成功")

    # 初始化 DynamicTFPublisher
    rei_tf = DynamicTFPublisher()
    rei_tf.clear_all_tf()

    # 桥接
    bridge = CvBridge()

    # 相机内参
    cam_matrix = np.zeros((3, 3), dtype=np.float64)
    dist_coeffs = np.zeros(5, dtype=np.float64)

    # 物块尺寸
    dims = {}
    for name in CLASS_NAMES.values():
        dims[name] = DEFAULT_DIMS.get(name, (0.05, 0.05, 0.02))
        w, h, d = dims[name]
        rospy.loginfo(f"  {name}: {w:.3f} x {h:.3f} x {d:.3f} m")

    # 订阅相机内参
    rospy.Subscriber('/hand_camera/camera_info', CameraInfo,
                     lambda msg: camera_info_cb(msg, cam_matrix, dist_coeffs),
                     queue_size=1)

    # 检测信息发布器 (供 Grab_2.cpp 触发 TF 查询)
    det_pub = rospy.Publisher('/yolo_seg_grasp/detection_info', String, queue_size=10)

    # 订阅相机图像
    rospy.Subscriber('/hand_camera/image_raw', Image,
                     lambda msg: image_cb(msg, bridge, model, rei_tf,
                                         cam_matrix, dist_coeffs, dims, det_pub),
                     queue_size=1, buff_size=2**24)

    rospy.loginfo("YOLO 分割节点就绪，等待图像...")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal(f"异常: {e}")
