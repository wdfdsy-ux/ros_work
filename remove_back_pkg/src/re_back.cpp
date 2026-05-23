#include <ros/ros.h>
#include "relative_move/SetRelativeMove.h"

ros::ServiceClient client;

bool set_relmove(float x, float y, float theta) {
    ROS_INFO("等待服务 /relative_move 启动...");
    client.waitForExistence();
    ROS_INFO("服务已连接！");
    relative_move::SetRelativeMove srv;
    srv.request.goal.x = x;
    srv.request.goal.y = y;
    srv.request.goal.theta = theta;
    srv.request.global_frame = "odom";

    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("向后移动成功：%s", srv.response.message.c_str());
            return true;
        } else {
            ROS_ERROR("向后移动失败：%s", srv.response.message.c_str());
            return false;
        }
    } else {
        ROS_ERROR("服务调用失败！");
        return false;
    }
}

int main(int argc, char** argv) {
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "remove_back_node");
    ros::NodeHandle nh;
    client = nh.serviceClient<relative_move::SetRelativeMove>("/relative_move");

    ROS_INFO("===== 执行向后相对位移 =====");
    if (!set_relmove(-0.18, 0, 0)) {
        return -1;
    }

    ROS_INFO("===== 向后位移完成 =====");
    return 0;
}
