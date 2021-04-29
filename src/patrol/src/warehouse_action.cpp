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

// Custom msg & srv
#include <detection_msgs/Detection2DTrig.h>
#include <detection_msgs/Det3D.h>
#include <detection_msgs/Det3DArray.h>

using namespace std;

class warehouse_action
{
private:
    std_msgs::Bool grip;
    char target_number, last_target_number = '0';
    const string PLANNING_GROUP = "tm_arm";
    const vector<double> home_p = {-M_PI_2, -M_PI_4, M_PI*2/3, -1.309, M_PI, 0.0};
    const vector<double> home_r = {M_PI, M_PI/3, -M_PI_4*3, 1.309, -M_PI_2, 0.0};
    const vector<double> middle1_a = {-M_PI, -M_PI_4, M_PI*2/3, -1.309, M_PI, 0.000};
    const vector<double> middle1_b = {-2.552, -0.527, 2.218, -1.679, 1.009, 0.000};
    const vector<double> middle2_a = {0, -M_PI_4, M_PI*2/3, -1.309, M_PI, 0.000};
    const vector<double> middle2_b = {3.344, -1.023, 2.259, -1.218, 1.285, 0.000};
    const vector<double> middler_p = {M_PI_2, -M_PI/3, M_PI_4*3, -1.309, -M_PI_2, 0.0};
    const vector<double> middlep_r = {0.000, 0.000, 0.000, 0.000, -M_PI_2, 0.0};
    const vector<double> position1 = {-3.133, 0.180, 2.279, -2.450, 1.545, 0.000};
    const vector<double> position2 = {-2.583, 0.183 ,2.462 ,-2.644, 1.019, 0.000};
    const vector<double> position3 = {-2.190, 0.317, 2.316 ,-2.607, 0.590, 0.000};
    const vector<double> position4 = {3.181, -0.120, 2.709, -2.586, M_PI_2, 0.000};
    const vector<double> position5 = {-1.591, 0.076, 2.699, -2.781, 0.000, 0.000};
    const vector<double> position_s = {0, M_PI/3, -M_PI_4*3, 1.309, 0.000, 0.000}; //M_PI_4*3
    const vector<double> scan_3 = {-0.305, -0.485, 1.473, -1.953, 0.286, 0.000};
    const vector<double> scan_2 = {-0.870, 0.014, 2.161, -2.185, 0.888, 0.000};
    const vector<double> scan_1 = {-1.564, 0.014, 2.261, -2.227, 1.550, 0.000};
    float x_tmp = 0, y_tmp = 0, z_tmp = 0;
    int count = 0;
    detection_msgs::Det3DArray target_bias;
public:
    warehouse_action(ros::NodeHandle nh);
    void det_callback(detection_msgs::Det3DArray msg);
    void Position_Manager();

    ros::Publisher gripper_pub;
    ros::Subscriber det_sub;
};

warehouse_action::warehouse_action(ros::NodeHandle nh)
{  
    ros::AsyncSpinner spinner(1); 
    spinner.start();
    gripper_pub = nh.advertise<std_msgs::Bool>("/gripper/cmd_gripper", 1);
    // det_sub = nh.subscribe("/scan_clustering_node/det3d_result", 1, &warehouse_action::det_callback, this);
    Position_Manager();
}

void warehouse_action::det_callback(detection_msgs::Det3DArray msg)
{
    if(count <= 10)
    {
        for(int i=0; i<msg.dets_list.size(); i++)
        {
            target_bias.dets_list[i].class_name = msg.dets_list[i].class_name;
            x_tmp += msg.dets_list[i].y;
            y_tmp += msg.dets_list[i].x;
            z_tmp += msg.dets_list[i].z;
        }
        count++;
    }
    else
    {

    }
}

void warehouse_action::Position_Manager()
{
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group.setStartState(*move_group.getCurrentState());
    
    ROS_INFO("GO TO TARGET");
    geometry_msgs::Pose target_pose1;
    target_pose1 = move_group.getCurrentPose().pose;
    // target_pose1.orientation.x = 0.707;
    // target_pose1.orientation.y = 0.000;
    // target_pose1.orientation.z = 0.000;
    // target_pose1.orientation.w = 0.707;
    // target_pose1.position.x = 0.280;  //(-)camera right
    // target_pose1.position.y = -0.200; //(-)camera forward
    // target_pose1.position.z = 0.800;  //(+)camera upward
    // target_pose1.position.x += 0.148 + 0.03;     //target_bias.position.y
    // target_pose1.position.y -= 0.100;       //target_bias.position.x
    // target_pose1.position.z += -0.048;      //target_bias.position.z
    // move_group.setPoseTarget(target_pose1);
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // move_group.move();
    // target_pose1.position.y -= 0.138 - 0.008;
    // move_group.setPoseTarget(target_pose1);
    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // move_group.move();
    // cout<<"Position"<<endl;
    // cout<<"X: "<<target_pose1.position.x<<endl;
    // cout<<"Y: "<<target_pose1.position.y<<endl;
    // cout<<"Z: "<<target_pose1.position.z<<endl;
    // cout<<"Rotation"<<endl;
    // cout<<"rx: "<<target_pose1.orientation.x<<endl;
    // cout<<"ry: "<<target_pose1.orientation.y<<endl;
    // cout<<"rz: "<<target_pose1.orientation.z<<endl;
    // cout<<"rw: "<<target_pose1.orientation.w<<endl;
    // last_target_number = 'g';

    while(1)
    {
        target_number = getchar();

        if(target_number == 'h')
        {
            ROS_INFO("GO HOME");

            if(last_target_number == '4')
            {
                joint_group_positions = middle2_b;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();

                joint_group_positions = middle2_a;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();

                joint_group_positions = home_p;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();
            }
            else if(last_target_number == '1' || last_target_number == '2' || last_target_number == '3' || last_target_number == '5')
            {
                joint_group_positions = middle1_b;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();

                joint_group_positions = home_p;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();
            }
            // else if(last_target_number == 's')
            // {
            //     joint_group_positions = middlep_r;
            //     move_group.setJointValueTarget(joint_group_positions);
            //     move_group.move();

            //     joint_group_positions = home_p;
            //     move_group.setJointValueTarget(joint_group_positions);
            //     move_group.move();
            // }
            else
            {
                joint_group_positions = home_p;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();
            }
            last_target_number = target_number;

            ROS_INFO("DONE");
        }
        if(target_number == '1')
        {
            ROS_INFO("GO FIRST POINT");
            grip.data = true;
            gripper_pub.publish(grip);
            sleep(1);

            if(last_target_number == 'h')
            {
                joint_group_positions = middle1_a;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();
            }
            if(last_target_number == 'g')
            {
                // target_pose1.position.y += 0.230;
                // move_group.setPoseTarget(target_pose1);
                // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                // move_group.move();

                joint_group_positions = position_s;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();

                joint_group_positions = middlep_r;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();               
            }

            ROS_INFO("MIDDLE POINT");

            joint_group_positions = middle1_a;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();

            joint_group_positions = middle1_b;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            joint_group_positions = position1;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            last_target_number = target_number;

            ROS_INFO("DONE");
            grip.data = false;
            gripper_pub.publish(grip);
        }
        if(target_number == '2')
        {
            ROS_INFO("GO SECOND POINT");

            joint_group_positions = middle1_a;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("MIDDLE POINT");

            joint_group_positions = middle1_b;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            joint_group_positions = position2;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            last_target_number = target_number;

            ROS_INFO("DONE");
        }
        if(target_number == '3')
        {
            ROS_INFO("GO THIRD POINT");

            joint_group_positions = middle1_a;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("MIDDLE POINT");

            joint_group_positions = middle1_b;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            joint_group_positions = position3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            last_target_number = target_number;

            ROS_INFO("DONE");
        }
        if(target_number == '4')
        {
            ROS_INFO("GO FORTH POINT");

            joint_group_positions = middle2_a;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("MIDDLE POINT");

            joint_group_positions = middle2_b;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            joint_group_positions = position4;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            last_target_number = target_number;

            ROS_INFO("DONE");
        }
        if(target_number == '5')
        {
            ROS_INFO("GO FIFTH POINT");

            joint_group_positions = middle1_a;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("MIDDLE POINT");

            joint_group_positions = middle1_b;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            joint_group_positions = position5;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            last_target_number = target_number;

            ROS_INFO("DONE");
        }
        if(target_number == 's')
        {
            ROS_INFO("GO SCANNING POINT 1");
            
            // joint_group_positions = scan_1;
            // move_group.setJointValueTarget(joint_group_positions);
            // move_group.move();

            ROS_INFO("GO SCANNING POINT 2");

            joint_group_positions = scan_2;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            // ROS_INFO("GO SCANNING POINT 3");

            // joint_group_positions = scan_3;
            // move_group.setJointValueTarget(joint_group_positions);
            // move_group.move();

            last_target_number = target_number;

            ROS_INFO("DONE");
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
    ros::init(argc, argv, "warehouse_action");
    ros::NodeHandle nh;
    warehouse_action node(nh);
    ros::spin();
    return 0;
}