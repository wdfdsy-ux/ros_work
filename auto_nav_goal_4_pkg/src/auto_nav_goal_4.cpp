
#include <ros/ros.h>
#include "relative_move/SetRelativeMove.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ar_pose/Track.h>
#include <tf/transform_listener.h>

// 创建服务客户端
ros::ServiceClient relmove_client;
ros::ServiceClient track_client;

// 定义 Action 客户端类型
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient* nav_client;

bool set_ARtrack(int id, float dist){
    // 等待服务上线
    ROS_INFO("等待服务 /track 启动...");
    track_client.waitForExistence();
    ROS_INFO("服务已连接！");
    ar_pose::Track srv;
    srv.request.ar_id = id;
    srv.request.goal_dist = dist;
    //发送请求
    if (track_client.call(srv)){
        if (srv.response.success){
            ROS_INFO("二次定位成功：%s", srv.response.message.c_str());
            return 1;
        }else{
            ROS_ERROR("二次定位失败：%s", srv.response.message.c_str());
            return 0;
        }
    }else{
        ROS_ERROR("track服务调用失败！");
        return 0;
    }
}

bool set_relmove(float x,float y,float theta){
    // 等待服务上线
    ROS_INFO("等待服务 /relative_move 启动...");
    relmove_client.waitForExistence();
    ROS_INFO("服务已连接！");
    // 定义服务消息
    relative_move::SetRelativeMove srv;

    // 填充请求数据
    srv.request.goal.x = x;
    srv.request.goal.y = y;
    srv.request.goal.theta = theta;
    srv.request.global_frame = "odom";
    // 发送请求
    if (relmove_client.call(srv)){
        if (srv.response.success){
            ROS_INFO("移动成功：%s", srv.response.message.c_str());
            return 1;
        }else{
            ROS_ERROR("移动失败：%s", srv.response.message.c_str());
            return 0;
        }
    }else{
        ROS_ERROR("服务调用失败！");
        return 0;
    }
}

bool navToGoal(double x, double y, double z, double w){
    // 等待服务器连接成功
    ROS_INFO("等待连接 move_base 服务器...");
    nav_client->waitForServer();
    ROS_INFO("连接成功！");
    nav_client->cancelAllGoals();
    ROS_WARN("已清空所有导航任务！");
    // 构造导航目标消息
    move_base_msgs::MoveBaseGoal goal;

    // 设置坐标系为 map
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // 设置目标坐标（可修改 x, y）
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    // 设置朝向
    goal.target_pose.pose.orientation.z = z;
    goal.target_pose.pose.orientation.w = w;
    // 发送目标点
    ROS_INFO("发送导航目标...");
    nav_client->sendGoal(goal);

    // 循环监听导航状态
    ros::Rate rate(5);
    while (ros::ok())
    {
        actionlib::SimpleClientGoalState state = nav_client->getState();
        std::string state_str = state.toString();

        // 实时打印状态
        ROS_INFO("当前导航状态：%s", state_str.c_str());

        // 导航成功
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("导航成功：已到达目标点！");
            return true;
        }
        // 导航失败（内部错误/障碍物/无法规划）
        else if (state == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_ERROR("导航失败：无法到达目标！");
            return false;
        }
        // 任务被取消
        else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
        {
            ROS_WARN("导航任务已被取消！");
            return false;
        }
        // 任务被拒绝
        else if (state == actionlib::SimpleClientGoalState::REJECTED)
        {
            ROS_ERROR("导航目标被服务器拒绝！");
            return false;
        }
        // 导航超时
        else if (state == actionlib::SimpleClientGoalState::LOST)
        {
            ROS_ERROR("导航连接丢失！");
            return false;
        }
        rate.sleep();
    }
    // ROS 退出
    return false;
}

bool get_transform(const std::string& parent_frame, const std::string& child_frame,
                   tf::StampedTransform& transform) {
    tf::TransformListener listener;
    try {
        listener.waitForTransform(parent_frame, child_frame, ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
        return true;
    } catch (tf::TransformException& ex) {
        ROS_ERROR("获取TF变换失败: %s", ex.what());
        return false;
    }
}

int main(int argc, char** argv) {
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "auto_nav_node");
    ros::NodeHandle nh;

    // 初始化服务客户端
    relmove_client = nh.serviceClient<relative_move::SetRelativeMove>("/relative_move");
    track_client = nh.serviceClient<ar_pose::Track>("/track");
    nav_client = new MoveBaseClient("move_base",true);

    try {
        if (!navToGoal(0.361, 2.034, 0.029, 1.000)) {
            return -1;
        }
        if (!set_ARtrack(0, 0.4)) {
            return -1;
        }
        if (!set_relmove(0.18, 0, 0)) {
            return -1;
        }

        ros::Duration(1.0).sleep();

        // if (!set_relmove(-0.18, 0, 0)) {
        //     return -1;
        // }

    } catch (ros::Exception& e) {
        ROS_ERROR("程序被中断：%s", e.what());
    }

    return 0;
}
