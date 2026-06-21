#include <ros/ros.h>
#include <arm_controller/move.h>

/*
 * 机械臂观测节点
 * 功能: 移动机械臂到观测位置，便于相机进行 YOLO 识别
 * 目标位置: (200, -150, 120)  单位 mm
 */

int main(int argc, char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "observation_node");
    ros::NodeHandle nh;

    // 目标观测位置 (mm)
    float target_x = 200.0f;
    float target_y = -150.0f;
    float target_z = 120.0f;

    // 等待服务启动
    ROS_INFO("Waiting for /goto_position service...");
    ros::ServiceClient client = nh.serviceClient<arm_controller::move>("/goto_position");
    client.waitForExistence();
    ROS_INFO("Service connected!");

    // 调用服务移动机械臂
    arm_controller::move srv;
    srv.request.pose.position.x = target_x;
    srv.request.pose.position.y = target_y;
    srv.request.pose.position.z = target_z;

    ROS_INFO("Moving to observation position (%.0f, %.0f, %.0f)...",
             target_x, target_y, target_z);

    if (client.call(srv))
    {
        if (srv.response.success)
        {
            ROS_INFO("OK — observation position reached");
            return 0;
        }
        else
        {
            ROS_ERROR("Failed: %s", srv.response.message.c_str());
            return 1;
        }
    }
    else
    {
        ROS_ERROR("Failed to call /goto_position service");
        return 1;
    }
}
