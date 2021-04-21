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

using namespace std;

class warehouse_action
{
private:
    std_msgs::Bool grip;
    char target_number, last_target_number = '0';
    const string PLANNING_GROUP = "tm_arm";
    const vector<double> home = {-M_PI_2, -M_PI_4, M_PI*2/3, -1.309, M_PI, 0.0};
    const vector<double> middle1_a = {-M_PI, -M_PI_4, M_PI*2/3, -1.309, M_PI, 0.000};
    const vector<double> middle1_b = {-2.552, -0.527, 2.218, -1.679, 1.009, 0.000};
    const vector<double> middle2_a = {0, -M_PI_4, M_PI*2/3, -1.309, M_PI, 0.000};
    const vector<double> middle2_b = {3.344, -1.023, 2.259, -1.218, 1.285, 0.000};
    const vector<double> position1 = {-3.133, 0.180, 2.279, -2.450, 1.545, 0.000};
    const vector<double> position2 = {-2.583, 0.183 ,2.462 ,-2.644, 1.019, 0.000};
    const vector<double> position3 = {-2.190, 0.317, 2.316 ,-2.607, 0.590, 0.000};
    const vector<double> position4 = {3.181, -0.120, 2.709, -2.586, M_PI_2, 0.000};
    const vector<double> position5 = {-1.591, 0.076, 2.699, -2.781, 0.002, 0.000};
public:
    warehouse_action(ros::NodeHandle nh);
    void Position_Manager();

    ros::Publisher gripper_pub;
};

warehouse_action::warehouse_action(ros::NodeHandle nh)
{  
    ros::AsyncSpinner spinner(1); 
    spinner.start();
    gripper_pub = nh.advertise<std_msgs::Bool>("/gripper/cmd_gripper", 1);
    Position_Manager();
}

void warehouse_action::Position_Manager()
{
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group.setStartState(*move_group.getCurrentState());
    
    while(true)
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

                joint_group_positions = home;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();
            }
            else if(last_target_number == '1' || last_target_number == '2' || last_target_number == '3' || last_target_number == '5')
            {
                joint_group_positions = middle1_b;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();

                joint_group_positions = home;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();
            }
            else
            {
                joint_group_positions = home;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();
            }
            last_target_number = target_number;

            ROS_INFO("DONE");
        }
        if(target_number == '1')
        {
            ROS_INFO("GO FIRST POINT");
            // grip.data = true;
            // gripper_pub.publish(grip);
            // sleep(1);
            joint_group_positions = middle1_a;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("MIDDLE POINT");

            joint_group_positions = middle1_b;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            joint_group_positions = position1;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            last_target_number = target_number;

            ROS_INFO("DONE");
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
        if(target_number == 'q' || target_number == 'Q')
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
    return 0;
}