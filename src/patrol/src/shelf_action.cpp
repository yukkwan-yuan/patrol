#include <iostream>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

//ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// Custom msg & srv
#include <detection_msgs/Detection2DTrig.h>
#include <detection_msgs/Det3D.h>
#include <detection_msgs/Det3DArray.h>

using namespace std;

class shelf_action
{
private:
    char target_number;
    const string PLANNING_GROUP = "tm_arm";
    const vector<double> home_p = {-M_PI_2, -M_PI_4, M_PI*2/3, -1.309, M_PI, 0.0};
    const vector<double> joint_sf_scan1 = {-1.564, 0.014, 2.261, -2.227, 1.550, 0.000};
    const vector<double> joint_sf_scan2 = {-0.870, 0.014, 2.161, -2.185, 0.888, 0.000};
    
    detection_msgs::Det3DArray target_bias;
    detection_msgs::Det3D bias;
    bool reach = false;
public:
    shelf_action(ros::NodeHandle nh);
    void det_callback(detection_msgs::Det3DArray msg);
    void Position_Manager();

    ros::Subscriber det_sub;
    geometry_msgs::Pose current_pose;
};

shelf_action::shelf_action(ros::NodeHandle nh)
{  
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    // det_sub = nh.subscribe("/scan_clustering_node/det3d_result", 1, &warehouse_action::det_callback, this);
    Position_Manager();
}

void shelf_action::det_callback(detection_msgs::Det3DArray msg)
{
    if(reach)
    {
        
    }
}

void shelf_action::Position_Manager()
{
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group.setStartState(*move_group.getCurrentState());

    while(1)
    {
        target_number = getchar();

        if(target_number == 'h')
        {
            ROS_INFO("GO HOME");

            joint_group_positions = home_p;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("DONE");
        }
        if(target_number == 's')
        {
            ROS_INFO("GO SCANNING POINT 1");
            
            joint_group_positions = joint_sf_scan1;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("DONE");

            sleep(1);
            reach = true;
        }
        if(target_number == 'q')
        {
            ROS_INFO("QUIT");
            break;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shelf_action");
    ros::NodeHandle nh;
    shelf_action node(nh);
    ros::spin();
    return 0;
}