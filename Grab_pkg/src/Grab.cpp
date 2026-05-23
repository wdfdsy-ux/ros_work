#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "arm_controller/move.h"
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"
#include <set>

ros::ServiceClient armmove_client;
ros::ServiceClient pick_client;
ros::ServiceClient put_client;
ros::Subscriber ar_sub;
bool tag_100_detected = false;
bool cargo_scan_done = false;
std::set<int> cargo_ids_detected;

bool arm_move(float x, float y, float z)
{
    ROS_INFO("等待服务 /goto_position 启动...");
    armmove_client.waitForExistence();
    ROS_INFO("服务已连接！");
    arm_controller::move srv;
    srv.request.pose.position.x = x;
    srv.request.pose.position.y = y;
    srv.request.pose.position.z = z;

    if (armmove_client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("机械臂移动成功：目标(%.2f, %.2f, %.2f)", x, y, z);
            return true;
        } else {
            ROS_ERROR("机械臂移动失败：%s", srv.response.message.c_str());
            return false;
        }
    } else {
        ROS_ERROR("机械臂服务调用失败！");
        return false;
    }
}

void set_pump(bool state)
{
    ROS_INFO("等待抓取服务启动...");
    pick_client.waitForExistence();
    put_client.waitForExistence();
    ROS_INFO("服务已连接！");
    std_srvs::Empty srv;
    if (state) {
        pick_client.call(srv);
        ROS_INFO("吸盘已开启");
    } else {
        put_client.call(srv);
        ROS_INFO("吸盘已关闭");
    }
}

void arMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& markers)
{
    for (const auto& marker : markers->markers) {
        // 检测定位标签 100
        if (marker.id == 100 && !tag_100_detected) {
            tag_100_detected = true;
            ROS_INFO("定位成功！已识别到AR标签 100");
        }
        // 检测货物标签 1~4
        if (marker.id >= 1 && marker.id <= 4 && tag_100_detected && !cargo_scan_done) {
            if (cargo_ids_detected.find(marker.id) == cargo_ids_detected.end()) {
                cargo_ids_detected.insert(marker.id);
                ROS_INFO("检测到货物AR标签 ID=%d", marker.id);
            }
        }
    }
}

int main(int argc, char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "grab_node");
    ros::NodeHandle nh;

    armmove_client = nh.serviceClient<arm_controller::move>("/goto_position");
    pick_client = nh.serviceClient<std_srvs::Empty>("/swiftpro/on");
    put_client = nh.serviceClient<std_srvs::Empty>("/swiftpro/off");

    // ===== Step 1: 机械臂移到安全点 =====
    ROS_INFO("===== Step 1: 移至安全点 (150, 0, 120) =====");
    if (!arm_move(150, 0, 120)) {
        return -1;
    }

    // ===== Step 2: 机械臂移到定位点 =====
    ROS_INFO("===== Step 2: 移至定位点 (165, -210, 70) =====");
    if (!arm_move(165, -210, 70)) {
        return -1;
    }

    // ===== Step 3: 被动识别定位标签 100 =====
    ROS_INFO("===== Step 3: 等待AR标签 100 识别... =====");
    ar_sub = nh.subscribe("hand_camera/ar_pose_marker", 10, arMarkerCallback);

    ros::Rate rate(10);
    while (ros::ok() && !tag_100_detected) {
        ros::spinOnce();
        rate.sleep();
    }
    if (!tag_100_detected) {
        ROS_ERROR("定位失败：未识别到AR标签 100");
        return -1;
    }

    // ===== Step 4: 机械臂移到货物识别点 =====
    ROS_INFO("===== Step 4: 移至货物识别点 (180, -180, 90) =====");
    if (!arm_move(180, -180, 90)) {
        return -1;
    }

    // ===== Step 5: 扫描货物标签 (ID: 1, 2, 3, 4) =====
    ROS_INFO("===== Step 5: 扫描货物AR标签 (ID:1~4) =====");
    ros::Time scan_start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - scan_start).toSec() < 3.0) {
        ros::spinOnce();
        rate.sleep();
    }
    cargo_scan_done = true;

    if (cargo_ids_detected.empty()) {
        ROS_ERROR("识别失败：未检测到任何货物标签 (ID:1~4)");
        return -1;
    }

    int min_id = *cargo_ids_detected.begin();
    std::string ids_str;
    for (int id : cargo_ids_detected) {
        ids_str += std::to_string(id) + " ";
    }
    ROS_INFO("共检测到 %zu 个货物标签 [%s]，最小ID=%d",
             cargo_ids_detected.size(), ids_str.c_str(), min_id);

    // ===== Step 6: 移动到最小ID货物标签正上方 40mm =====
    ROS_INFO("===== Step 6: 移至货物标签 %d 上方40mm =====", min_id);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    std::string target_frame = "ar_marker_" + std::to_string(min_id);

    try {
        listener.waitForTransform("Base", target_frame, ros::Time(0), ros::Duration(5.0));
        listener.lookupTransform("Base", target_frame, ros::Time(0), transform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("获取货物标签TF坐标失败: %s", ex.what());
        return -1;
    }

    float cargo_x = transform.getOrigin().x() * 1000;
    float cargo_y = transform.getOrigin().y() * 1000;
    float cargo_z_surface = transform.getOrigin().z() * 1000;  // 货物表面
    float tag_z = cargo_z_surface + 40;  // 上方40mm

    ROS_INFO("货物标签位置: (%.2f, %.2f, %.2f)mm", cargo_x, cargo_y, cargo_z_surface);
    ROS_INFO("目标位置(上方40mm): (%.2f, %.2f, %.2f)mm", cargo_x, cargo_y, tag_z);

    if (!arm_move(cargo_x, cargo_y, tag_z)) {
        return -1;
    }

    // ===== Step 7: 抓取货物 =====
    ROS_INFO("===== Step 7: 抓取货物 =====");
    // 下降到货物表面
    arm_move(cargo_x, cargo_y, cargo_z_surface);
    ros::Duration(1.0).sleep();
    // 开启吸盘
    set_pump(true);
    // 抬起
    arm_move(cargo_x, cargo_y, tag_z);

    // ===== Step 8: 放置货物（选择出刀点）=====
    bool point1_occupied = false;
    nh.param("placement_point_1_occupied", point1_occupied, false);

    if (!point1_occupied) {
        // 出刀点1 未被占用 → 使用出刀点1
        ROS_INFO("===== Step 8: 移至出刀点1 (107, 115, 42) =====");
        arm_move(107, 115, 92);   // 升至出刀点上方50mm处
        ros::Duration(1.0).sleep();
        arm_move(107, 115, 42);   // 缓慢下移至放置点
        ros::Duration(1.0).sleep();
        set_pump(false);           // 关闭吸盘，松开货物
        arm_move(107, 115, 52);   // 出刀
        // 标记出刀点1 为已占用
        nh.setParam("placement_point_1_occupied", true);
        ROS_WARN("出刀点1 已标记为占用");
    } else {
        // 出刀点1 已被占用 → 使用出刀点2
        ROS_INFO("===== Step 8: 出刀点1已被占用，移至出刀点2 (107, 185, 42) =====");
        arm_move(107, 185, 92);   // 升至出刀点上方50mm处
        ros::Duration(1.0).sleep();
        arm_move(107, 185, 42);   // 缓慢下移至放置点
        ros::Duration(1.0).sleep();
        set_pump(false);           // 关闭吸盘，松开货物
        arm_move(107, 185, 52);   // 出刀
    }

    // ===== Step 9: 回到安全点 =====
    ROS_INFO("===== Step 9: 回到安全点 (150, 0, 120) =====");
    if (!arm_move(150, 0, 120)) {
        return -1;
    }

    ROS_INFO("===== 全部完成！=====");
    return 0;
}
