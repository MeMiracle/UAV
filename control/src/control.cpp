#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>
#include <Eigen/Core>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

ros::Publisher trajectory_pub;
geometry_msgs::PointStamped current_position;

float linear_smoothing_navigation_step = 2;
bool flag_gps_initialized_OK = false;
bool flag_take_off_OK = false;
int flag_tasks_OK = 0;
Eigen::Vector3d home;

/*
Description: 
    updateUavPosition(const geometry_msgs::PointStamped& msg)
    gps数据更新的回调函数.

Parameters:
    msg 位置信息

Return:
    无
*/
void updateUavPosition(const geometry_msgs::PointStamped& msg)
{
    if (!flag_gps_initialized_OK)
    {
        flag_gps_initialized_OK= true;
        home[0] = msg.point.x;
        home[1] = msg.point.y;
        home[2] = msg.point.z;
    }
    current_position = msg;
    // std::cout<<"UAV current position is: "<<msg.point.x<< msg.point.y<< msg.point.z<<std::endl;
}

/*
Description: 
    getDistanceToTarget(const Eigen::Vector3d& target)
    获取当前位置到指定位置位置的距离.

Parameters:
    target 需要飞达的位置点

Return:
    double 当前位置到达目标点的位置
*/
double getDistanceToTarget(const Eigen::Vector3d& target)
{
    double temp = 0;
    temp += pow((target[0] - current_position.point.x), 2);
	temp += pow((target[1] - current_position.point.y), 2);
	temp += pow((target[2] - current_position.point.z), 2);
    temp = sqrt(temp);
    return temp;
}

/*
Description: 
    reachTargetPosition(const Eigen::Vector3d& target, float max_error)
    判定是否到达指定的目标点.

Parameters:
    target 需要飞达的位置点
    max_error 允许的位置误差阈值,当前位置和目标位置小于该阈值时,判定无人机到达目标点

Return:
    bool 到达目标点时返回 true
         未到达目标点时返回 false
*/
bool reachTargetPosition(const Eigen::Vector3d& target, float max_error)
{
    double temp = getDistanceToTarget(target);
	
	if (temp < max_error)
		return true;
	return false;
}

/*
Description: 
    linearSmoothingNavigationTask(const Eigen::Vector3d& target)
    控制无人机从当前位置飞向指定位置.

Parameters:
    target 需要飞达的位置点

Return:
    bool 起飞结束后返回 true
         起飞过程中返回 false
*/
bool linearSmoothingNavigationTask(const Eigen::Vector3d& target)
{
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();

    if (reachTargetPosition(target,0.2))
        return true;

    double dist = getDistanceToTarget(target);
    Eigen::Vector3d next_step;
    
    if(dist<linear_smoothing_navigation_step)
    {
        next_step = target;
    }
    else
    {
        next_step[0] = current_position.point.x+(target[0]-current_position.point.x)/dist*linear_smoothing_navigation_step;
        next_step[1] = current_position.point.y+(target[1]-current_position.point.y)/dist*linear_smoothing_navigation_step;
        next_step[2] = current_position.point.z+(target[2]-current_position.point.z)/dist*linear_smoothing_navigation_step;
    }
    
    double desired_yaw = 0.0; 
    
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(next_step, desired_yaw, &trajectory_msg);
    trajectory_pub.publish(trajectory_msg);
    return false;
}

/*
Description: 
    takeOffTask(float height)
    起飞函数,调用后无人机从起始位置起飞指定高度.

Parameters:
    height 指定的起飞高度

Return:
    bool 起飞结束后返回 true
         起飞过程中返回 false
*/
bool takeOffTask(float height)
{
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();

    static Eigen::Vector3d desired_position(current_position.point.x, current_position.point.y, height);
    double desired_yaw = 0.0;

    if (reachTargetPosition(desired_position,0.2))
        return true;
    
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
    trajectory_pub.publish(trajectory_msg);
    return false;
}

/*
Description: 
    gohome()
    反航函数,调用后无人机先沿着当前高度飞到反航点正上方,然后降落.

Parameters:
    无

Return:
    无
*/
void gohome()
{
    static Eigen::Vector3d temp(home[0], home[1], current_position.point.z);
    static bool flag_temp = false;
    
    if (!flag_temp)
    {
        flag_temp = linearSmoothingNavigationTask(temp);
    }
    else
    {
        linearSmoothingNavigationTask(home);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "UAV_Controler");
    ros::NodeHandle nh;
    // Create a private node handle for accessing node parameters.
    ros::NodeHandle nh_private("~");

    std::string uav_name = "";  
    ros::param::get("~mav_name",uav_name);

    // 订阅话题
    // /odometry_sensor1/position   无人机位置信息(包含噪声)
    ros::Subscriber position_sub = nh.subscribe(std::string("/"+uav_name+"/odometry_sensor1/position").c_str(), 10, &updateUavPosition);

    trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

    // 等待5s,让Gazebo可以成功启动.
    ros::Duration(5.0).sleep();

    // 创建控制Gazebo自动运行的服务,这里自动运行是指让Gazebo自动Play
    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv); 

    // 尝试让Gazebo自动运行
    int i=0;
    while (i <= 10 && !unpaused) {
        ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
        ++i;
    }

    // 判断Gazebo有没有自动运行,没有成功Play则退出
    if (!unpaused) {
        ROS_FATAL("Could not wake up Gazebo.");
        return -1;
    } else {
        ROS_INFO("Unpaused the Gazebo simulation.");
    }

    std::vector<Eigen::Vector3d> path;
    path.push_back(Eigen::Vector3d(5.f,5.f,5.f));
    path.push_back(Eigen::Vector3d(-5.f,5.f,5.f));
    path.push_back(Eigen::Vector3d(-5.f,-5.f,5.f));
    path.push_back(Eigen::Vector3d(5.f,-5.f,5.f));
    path.push_back(Eigen::Vector3d(5.f,5.f,5.f));
    std::cout << path.size() << std::endl;

    ros::Rate loop_rate(10);
    while (ros::ok())
    { 
        if(flag_gps_initialized_OK && !flag_take_off_OK)
        {
            // ROS_INFO("UAV take off task is running...");
            flag_take_off_OK = takeOffTask(3);
        }
        else if(flag_take_off_OK && flag_tasks_OK<path.size())
        {
            if(flag_tasks_OK<path.size())
            {
                
                bool temp = linearSmoothingNavigationTask(path[flag_tasks_OK]);
                if (temp)
                    flag_tasks_OK ++;
            }
        }
        else if(flag_tasks_OK >= path.size())
        {
            gohome();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}