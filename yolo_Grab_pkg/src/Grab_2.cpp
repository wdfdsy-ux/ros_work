#include <ros/ros.h>
#include <arm_controller/move.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

/*
 * YOLOv8-seg 视觉抓取机械臂控制节点
 *
 * 通信协议 (与 split.py 配合):
 *   split.py 发布 TF:   parent_frame → object_{class}_{id}
 *   split.py 发布话题:  /yolo_seg_grasp/detection_info  (String)
 *                        格式:  id|class|conf|x,y,z|qx,qy,qz,qw
 *
 * 工作流程:
 *   1. 订阅 detection_info 获取识别到的物块
 *   2. 通过 TF 将物块位姿从相机坐标系转换到 Base 坐标系
 *   3. 调用机械臂服务移动
 *
 * 机械臂移动序列:
 *   Step 1: 安全点 (150, 0, 120)      — 单位 mm
 *   Step 2: 物块上方40mm               — 基于 TF 坐标
 *   Step 3: 下降至表面 +10mm 抓取
 *   Step 4: 吸盘吸取 → 抬起 → 放置 → 释放
 */

class YoloGrabController
{
public:
    YoloGrabController(ros::NodeHandle& nh) : nh_(nh), detection_received_(false), conf_threshold_(0.0f)
    {
        // ========== 服务客户端 ==========
        armmove_client_ = nh_.serviceClient<arm_controller::move>("/goto_position");
        pick_client_    = nh_.serviceClient<std_srvs::Empty>("/swiftpro/on");
        put_client_     = nh_.serviceClient<std_srvs::Empty>("/swiftpro/off");

        // ========== TF 监听器 ==========
        tf_listener_ = new tf::TransformListener();

        // ========== 参数 ==========
        nh_.param("detection_topic", detection_topic_,
                  std::string("/yolo_seg_grasp/detection_info"));
        nh_.param("parent_frame",    parent_frame_,    std::string("Base"));
        nh_.param("target_class",    target_class_,    std::string(""));  // ""=任意
        nh_.param("ignored_classes", ignored_classes_,
                  std::string("")); // 默认不过滤
        nh_.param("conf_threshold",  conf_threshold_,  0.0f);              // 0=不过滤
        nh_.param("safe_x", safe_x_, 200.0f);
        nh_.param("safe_y", safe_y_, -150.0f);
        nh_.param("safe_z", safe_z_, 100.0f);
        nh_.param("approach_offset", approach_offset_, 40.0f);  // 上方 mm
        nh_.param("grasp_offset",    grasp_offset_,    35.0f);  // 表面上方 mm
        nh_.param("lift_offset",     lift_offset_,     50.0f);  // 抓取后抬升 mm
        nh_.param("detection_timeout", detection_timeout_, 30.0);  // 秒

        // ========== 校准偏移 (补偿 TF 偏差) ==========
        nh_.param("offset_x", offset_x_, 0.0f);
        nh_.param("offset_y", offset_y_, 0.0f);
        nh_.param("offset_z", offset_z_, 0.0f);

        // ========== 出刀点参数 ==========
        nh_.param("place1_x", place1_x_, 107.0f);
        nh_.param("place1_y", place1_y_, 115.0f);
        nh_.param("place1_z", place1_z_, 42.0f);
        nh_.param("place2_x", place2_x_, 107.0f);
        nh_.param("place2_y", place2_y_, 185.0f);
        nh_.param("place2_z", place2_z_, 42.0f);

        // ========== 订阅检测信息 ==========
        det_sub_ = nh_.subscribe(detection_topic_, 10,
                                 &YoloGrabController::detectionCallback, this);

        ROS_INFO("===== YOLO Grab Controller Ready =====");
        ROS_INFO("  detection_topic: %s", detection_topic_.c_str());
        ROS_INFO("  parent_frame:    %s", parent_frame_.c_str());
        ROS_INFO("  target_class:    %s", target_class_.empty() ? "(any)" : target_class_.c_str());
        ROS_INFO("  ignored_classes: %s", ignored_classes_.c_str());
        ROS_INFO("  conf_threshold:  %.2f", conf_threshold_);
        ROS_INFO("  offset (x,y,z):  (%.1f, %.1f, %.1f)", offset_x_, offset_y_, offset_z_);
        ROS_INFO("  safe point:      (%.0f, %.0f, %.0f)", safe_x_, safe_y_, safe_z_);
        ROS_INFO("  approach offset: %.0f mm", approach_offset_);
        ROS_INFO("  grasp offset:    %.0f mm", grasp_offset_);
        ROS_INFO("  lift offset:     %.0f mm", lift_offset_);
        ROS_INFO("  place point 1:   (%.0f, %.0f, %.0f)", place1_x_, place1_y_, place1_z_);
        ROS_INFO("  place point 2:   (%.0f, %.0f, %.0f)", place2_x_, place2_y_, place2_z_);
    }

    ~YoloGrabController()
    {
        delete tf_listener_;
    }

    // ======================================================================
    // 检测信息回调
    // ======================================================================
    void detectionCallback(const std_msgs::String::ConstPtr& msg)
    {
        if (detection_received_) return;  // 只取第一帧

        // 解析多行，取第一行
        std::string data = msg->data;
        if (data.empty()) return;

        size_t end = data.find('\n');
        std::string first = (end == std::string::npos) ? data : data.substr(0, end);

        // 格式: id|class|conf|x,y,z|qx,qy,qz,qw
        std::vector<std::string> tokens;
        std::stringstream ss(first);
        std::string token;
        while (std::getline(ss, token, '|'))
            tokens.push_back(token);

        if (tokens.size() < 4)
        {
            ROS_WARN("Malformed detection info: %s", first.c_str());
            return;
        }

        int id = std::stoi(tokens[0]);
        std::string cls = tokens[1];
        float conf = std::stof(tokens[2]);

        // 如果指定了 target_class，只接受匹配的类别
        if (!target_class_.empty() && cls != target_class_)
            return;

        // 过滤忽略列表中的类别（如 camera、person 等不可抓取物）
        if (isInIgnoredClasses(cls))
        {
            ROS_WARN("Ignored class detected: %s (conf=%.2f)", cls.c_str(), conf);
            return;
        }

        // 置信度过低则忽略
        if (conf < conf_threshold_)
        {
            ROS_WARN("Low confidence: %s conf=%.2f < threshold=%.2f",
                     cls.c_str(), conf, conf_threshold_);
            return;
        }

        detected_id_    = id;
        detected_class_ = cls;
        detected_frame_ = "object_" + detected_class_ + "_" + std::to_string(detected_id_);
        detection_received_ = true;

        if (tokens.size() >= 5)
            detected_conf_ = std::stof(tokens[2]);

        ROS_INFO(">>> Detection locked: frame=%s  class=%s  id=%d  conf=%.2f",
                 detected_frame_.c_str(), detected_class_.c_str(),
                 detected_id_, detected_conf_);
    }

    // ======================================================================
    // 等待检测 (阻塞)
    // ======================================================================
    bool waitForDetection()
    {
        ros::Time start = ros::Time::now();
        ros::Rate rate(30);

        ROS_INFO("Waiting for YOLO detection... (timeout=%.1fs)", detection_timeout_);

        while (ros::ok())
        {
            ros::spinOnce();

            if (detection_received_)
                return true;

            if ((ros::Time::now() - start).toSec() >= detection_timeout_)
            {
                ROS_ERROR("Detection timeout after %.1f seconds", detection_timeout_);
                return false;
            }

            rate.sleep();
        }

        return false;
    }

    // ======================================================================
    // TF 查询: 将物块位姿转换到 Base 坐标系
    // ======================================================================
    bool getObjectPoseInBase(tf::StampedTransform& out_transform)
    {
        try
        {
            ROS_INFO("Waiting for TF: %s → %s ...",
                     parent_frame_.c_str(), detected_frame_.c_str());

            tf_listener_->waitForTransform(parent_frame_, detected_frame_,
                                           ros::Time(0), ros::Duration(5.0));
            tf_listener_->lookupTransform(parent_frame_, detected_frame_,
                                          ros::Time(0), out_transform);

            return true;
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("TF lookup failed: %s", ex.what());
            return false;
        }
    }

    // ======================================================================
    // 调用机械臂服务 /goto_position (坐标单位: mm)
    // ======================================================================
    bool isInWorkspace(float x, float y, float z)
    {
        // 工作空间边界 (根据实际机械臂调整)
        const float x_min = 50.0f,  x_max = 300.0f;
        const float y_min = -300.0f, y_max = 300.0f;
        const float z_min = 20.0f,  z_max = 250.0f;
        return (x >= x_min && x <= x_max &&
                y >= y_min && y <= y_max &&
                z >= z_min && z <= z_max);
    }

    bool armMove(float x, float y, float z)
    {
        if (!isInWorkspace(x, y, z))
        {
            ROS_ERROR("Target (%.2f, %.2f, %.2f) is outside workspace!", x, y, z);
            return false;
        }
        ROS_INFO("Waiting for /goto_position service...");
        armmove_client_.waitForExistence();
        ROS_INFO("Service connected!");

        arm_controller::move srv;
        srv.request.pose.position.x = x;
        srv.request.pose.position.y = y;
        srv.request.pose.position.z = z;

        if (armmove_client_.call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO("OK — moved to (%.2f, %.2f, %.2f)", x, y, z);
                return true;
            }
            else
            {
                ROS_ERROR("Arm move failed: %s", srv.response.message.c_str());
                return false;
            }
        }
        else
        {
            ROS_ERROR("Failed to call /goto_position service!");
            return false;
        }
    }

    // ======================================================================
    // 过滤：检查类别是否在忽略列表中
    // ======================================================================
    bool isInIgnoredClasses(const std::string& cls)
    {
        if (ignored_classes_.empty()) return false;

        // 用逗号分割忽略列表
        std::string copy = ignored_classes_;
        std::stringstream ss(copy);
        std::string item;
        while (std::getline(ss, item, ','))
        {
            // 去除前后空格
            size_t start = item.find_first_not_of(" \t");
            size_t end   = item.find_last_not_of(" \t");
            std::string trimmed = (start == std::string::npos) ? "" :
                                  item.substr(start, end - start + 1);
            if (trimmed == cls)
                return true;
        }
        return false;
    }

    // ======================================================================
    // 吸盘控制
    // ======================================================================
    void setPump(bool on)
    {
        ROS_INFO("Waiting for pump service...");
        std_srvs::Empty srv;

        if (on)
        {
            pick_client_.waitForExistence();
            pick_client_.call(srv);
            ROS_INFO("Pump ON (suction activated)");
        }
        else
        {
            put_client_.waitForExistence();
            put_client_.call(srv);
            ROS_INFO("Pump OFF (release)");
        }
    }

    // ======================================================================
    // 安全收回
    // ======================================================================
    void safeRetract()
    {
        ROS_WARN("=== Safe retract → (%.0f, %.0f, %.0f) ===", safe_x_, safe_y_, safe_z_);
        armMove(safe_x_, safe_y_, safe_z_);
    }

    // ======================================================================
    // 主流程
    // ======================================================================
    void run()
    {
        // ===== Step 0: 先回到观测点 =====
        ROS_INFO("===== Step 0: Init → Observation point (%.0f, %.0f, %.0f) =====",
                 safe_x_, safe_y_, safe_z_);
        if (!armMove(safe_x_, safe_y_, safe_z_))
            return;

        // ===== Step 1: 等待 YOLO 检测结果 =====
        ROS_INFO("===== Step 1: Wait for YOLO detection =====");
        if (!waitForDetection())
        {
            safeRetract();
            return;
        }

        // ===== Step 2: 获取物块在 Base 坐标系下的位姿 =====
        ROS_INFO("===== Step 2: Get object pose in %s frame =====",
                 parent_frame_.c_str());
        tf::StampedTransform obj_tf;
        if (!getObjectPoseInBase(obj_tf))
        {
            safeRetract();
            return;
        }

        // 单位转换: TF 使用米 → 机械臂服务使用毫米
        float obj_x = obj_tf.getOrigin().x() * 1000.0f;
        float obj_y = obj_tf.getOrigin().y() * 1000.0f;
        float obj_z = obj_tf.getOrigin().z() * 1000.0f;

        ROS_INFO("Object in %s: (%.2f, %.2f, %.2f) mm",
                 parent_frame_.c_str(), obj_x, obj_y, obj_z);

        // 应用校准偏移
        obj_x += offset_x_;
        obj_y += offset_y_;
        obj_z += offset_z_;
        ROS_INFO("After offset (%.1f, %.1f, %.1f): (%.2f, %.2f, %.2f) mm",
                 offset_x_, offset_y_, offset_z_, obj_x, obj_y, obj_z);

        // Z 为负时记录警告（可能 TF 偏移），但仍尝试抓取，grasp_z 有下限保护
        if (obj_z < 0.0f)
        {
            ROS_WARN("Object Z=%.2f is negative — TF may be offset, continuing...", obj_z);
        }

        // ===== Step 3: 移动到物块上方 (保持观测高度 120mm 水平平移) =====
        // 避免使用 PnP 解算的 Z (可能为负值), 以安全高度先到达物块正上方
        float approach_z = safe_z_;  // 沿用观测高度作为安全接近高度
        ROS_INFO("===== Step 3: Approach above object → (%.2f, %.2f, %.2f) =====",
                 obj_x, obj_y, approach_z);
        if (!armMove(obj_x, obj_y, approach_z))
        {
            safeRetract();
            return;
        }

        // ===== Step 4: 下降至抓取高度 (取物块 Z 与下限值中的较大值) =====
        float grasp_z = std::max(obj_z + grasp_offset_, 30.0f);
        ROS_INFO("===== Step 4: Descend to grasp → (%.2f, %.2f, %.2f) =====",
                 obj_x, obj_y, grasp_z);
        if (!armMove(obj_x, obj_y, grasp_z))
        {
            safeRetract();
            return;
        }

        // ===== Step 5: 开启吸盘 =====
        ROS_INFO("===== Step 5: Activate suction =====");
        setPump(true);
        ros::Duration(0.5).sleep();

        // ===== Step 6: 垂直上升 (抓取后抬起) =====
        float lift_z = grasp_z + lift_offset_;
        ROS_INFO("===== Step 6: Lift vertically → (%.2f, %.2f, %.2f) =====",
                 obj_x, obj_y, lift_z);
        if (!armMove(obj_x, obj_y, lift_z))
        {
            safeRetract();
            return;
        }

        // ===== Step 7: 放置到出刀点 =====
        bool point1_occupied = false;
        bool point2_occupied = false;
        nh_.param("placement_point_1_occupied", point1_occupied, false);
        nh_.param("placement_point_2_occupied", point2_occupied, false);

        if (!point1_occupied)
        {
            // 出刀点1 空闲 → 使用点1
            ROS_INFO("===== Step 7: Place at point 1 → (%.0f, %.0f, %.0f) =====",
                     place1_x_, place1_y_, lift_z);
            if (!armMove(place1_x_, place1_y_, lift_z))
            {
                safeRetract();
                return;
            }

            ROS_INFO("Descend to placement height → (%.0f, %.0f, %.0f)",
                     place1_x_, place1_y_, place1_z_);
            if (!armMove(place1_x_, place1_y_, place1_z_))
            {
                safeRetract();
                return;
            }

            ros::Duration(1.0).sleep();
            setPump(false);

            // 出刀（向上抬起防撞）
            armMove(place1_x_, place1_y_, place1_z_ + 30.0f);

            nh_.setParam("placement_point_1_occupied", true);
            ROS_WARN("Placement point 1 marked as occupied");
        }
        else if (!point2_occupied)
        {
            // 出刀点1 已占用，点2 空闲 → 使用点2
            ROS_INFO("===== Step 7: Point 1 occupied, place at point 2 → (%.0f, %.0f, %.0f) =====",
                     place2_x_, place2_y_, lift_z);
            if (!armMove(place2_x_, place2_y_, lift_z))
            {
                safeRetract();
                return;
            }

            ROS_INFO("Descend to placement height → (%.0f, %.0f, %.0f)",
                     place2_x_, place2_y_, place2_z_);
            if (!armMove(place2_x_, place2_y_, place2_z_))
            {
                safeRetract();
                return;
            }

            ros::Duration(1.0).sleep();
            setPump(false);

            // 出刀（向上抬起防撞）
            armMove(place2_x_, place2_y_, place2_z_ + 30.0f);

            nh_.setParam("placement_point_2_occupied", true);
            ROS_WARN("Placement point 2 marked as occupied");
        }
        else
        {
            // 两个出刀点均已占用 → 报警退出
            ROS_ERROR("Both placement points are occupied! Can't place object.");
            safeRetract();
            return;
        }

        // ===== Step 8: 返回安全点 (150, 0, 120) =====
        ROS_INFO("===== Step 8: Return to safe point (150, 0, 120) =====");
        if (!armMove(150.0f, 0.0f, 120.0f))
            return;

        ROS_INFO("===== ALL DONE — grasp & place cycle complete =====");
    }

private:
    ros::NodeHandle& nh_;

    // 服务客户端
    ros::ServiceClient armmove_client_;
    ros::ServiceClient pick_client_;
    ros::ServiceClient put_client_;

    // TF 监听
    tf::TransformListener* tf_listener_;

    // 检测信息订阅
    ros::Subscriber det_sub_;

    // --- 可配置参数 ---
    std::string detection_topic_;
    std::string parent_frame_;
    std::string target_class_;
    float safe_x_, safe_y_, safe_z_;
    float approach_offset_;      // 物块上方距离 (mm)
    float grasp_offset_;         // 物块表面上方距离 (mm)
    float lift_offset_;          // 抓取后抬升距离 (mm)
    double detection_timeout_;   // 等待检测超时 (秒)

    // --- 出刀点坐标 ---
    float place1_x_, place1_y_, place1_z_;
    float place2_x_, place2_y_, place2_z_;

    // --- 校准偏移 ---
    float offset_x_, offset_y_, offset_z_;

    // --- 过滤参数 ---
    std::string ignored_classes_;    // 逗号分隔，默认 "camera,person,..."
    float       conf_threshold_;     // 置信度阈值

    // --- 检测结果 ---
    bool   detection_received_;
    int    detected_id_;
    std::string detected_class_;
    std::string detected_frame_;
    float  detected_conf_;
};


int main(int argc, char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "yolo_grab_controller");
    ros::NodeHandle nh("~");

    YoloGrabController controller(nh);
    controller.run();

    return 0;
}
