#include <iostream>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <termios.h> 
#include <unistd.h> 
#include <stdio.h>  
#include <vector>
//ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h> 
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
// Custom msg & srv
#include <detection_msgs/Detection2DTrig.h>
#include <detection_msgs/Det3D.h>
#include <detection_msgs/Det3DArray.h>
#include <detection_msgs/StringArray.h>


using namespace std;
geometry_msgs::Pose item_position,midlePose;
ros::Subscriber sub_item_position;
class nasal_swab
{
private:
    std_msgs::Bool grip;
    char command;
    const string PLANNING_GROUP = "tm_arm";
    const vector<double> wait_for_swab = {-0.01820257492363453, 0.1478414684534073, 2.1385350227355957, -2.4705910682678223, 1.6482404470443726, 0.013351768255233765};
    const vector<double> home_test = {-0.018215017393231392, 0.011714097112417221, 2.5290753841400146, -2.8384482860565186, 1.6483471393585205, 0.013409946113824844};
    const vector<double> home_gs = {-1.6396251916885376, -0.5873907208442688, 1.671115756034851, 0.522377073764801, 1.6284018754959106, -0.15363746881484985};
    const vector<double> home_p = {-0.017194775864481926, 0.23691341280937195, 2.100027084350586, -2.472792148590088, 1.6404544115066528, 0.012149429880082607};
    const vector<double> new_home_point = {-0.04551265761256218, -0.4030570089817047, 1.826123833656311, -1.283854603767395, 1.6480659246444702, 0.0};
    float *x_tmp, *y_tmp, *z_tmp;
    int count = 0, target_amount = 0, target_number = 0;
    std_msgs::Int8 replacement_finished;
    detection_msgs::Det3DArray target_bias;
    detection_msgs::Det3D bias;
    bool find = false;
    bool grab = false;
    bool collect = false;
    bool reach = false;

public:
    nasal_swab(ros::NodeHandle nh);
    void det_callback(detection_msgs::Det3DArray msg);
    void Position_Manager();
    //void mis_callback(detection_msgs::StringArray msg);
    //void loc_callback(std_msgs::Int8 msg);
    ros::Publisher gripper_pub;
    //ros::Publisher mis_pub;
    ros::Subscriber det_sub;
    ros::Subscriber mis_sub;
    ros::Subscriber loc_sub;
    geometry_msgs::Pose current_pose, joint_pose, ready_pose, target_pose;
    detection_msgs::StringArray mis_list, sa, prod_list;
};

nasal_swab::nasal_swab(ros::NodeHandle nh)
{  
    ros::AsyncSpinner spinner(1); 
    spinner.start();
    
    // x_tmp = new float[20] ();
    // y_tmp = new float[20] ();
    // z_tmp = new float[20] ();
    //mis_sub = nh.subscribe("/missing_bottle", 1, &ArmControl::mis_callback, this);
    gripper_pub = nh.advertise<std_msgs::Bool>("/gripper/cmd_gripper", 1);
    //mis_pub = nh.advertise<detection_msgs::Det3DArray>("/missing_bottle", 1);
    det_sub = nh.subscribe("/scan_nose_clustering_node/det3d_result", 1, &nasal_swab::det_callback, this);
    Position_Manager();
}

// base link frame
// left  +y   right -y
// front +x   back  -x
// up    +z   down  -z

int getch(void) 
{ 
    struct termios oldattr, newattr; 
    int ch; 
    tcgetattr(STDIN_FILENO, &oldattr); 
    newattr = oldattr; 
    newattr.c_lflag &= ~(ICANON | ECHO); 
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr); 
    ch = getchar(); 
    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr); 
    return ch; 
} 

/* reads from keypress, echoes */ 
int getche(void) 
{ 
    struct termios oldattr, newattr; 
    int ch; 
    tcgetattr(STDIN_FILENO, &oldattr); 
    newattr = oldattr; 
    newattr.c_lflag &= ~(ICANON); 
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr); 
    ch = getchar(); 
    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr); 
    return ch; 
} 

void nasal_swab::det_callback(detection_msgs::Det3DArray msg)
{
    if(msg.dets_list.size() != 0 && reach )
        find = true;
    else
        find = false;

    if(find && !collect)
    {
        target_bias.dets_list.clear();

        while(count < 20)
        {
            for(int i=0; i<msg.dets_list.size(); i++)
            {
                x_tmp[i] += msg.dets_list[i].x;
                y_tmp[i] += msg.dets_list[i].y;
                z_tmp[i] += msg.dets_list[i].z;
                //cout << x_tmp[i] << endl;
            }
            count++;
        }

        count = 0;

        for(int i=0; i<msg.dets_list.size(); i++)
        {
           //cout << "123"<< endl;
            //for(int j=0; j<mis_list.strings.size(); j++)
            //{
                //cout << "456"<< endl;
            //if(mis_list.strings[j] == msg.dets_list[i].class_name)
            //{
            bias.class_name = msg.dets_list[i].class_name;
            cout<<bias.class_name<<endl;
            bias.class_id = msg.dets_list[i].class_id;
            bias.x = x_tmp[i]/20;
            bias.y = y_tmp[i]/20;
            bias.z = z_tmp[i]/20;
            x_tmp[i] = 0;
            y_tmp[i] = 0;
            z_tmp[i] = 0;
            // cout << bias.x << endl;
            // cout << bias.y << endl;
            // cout << bias.z << endl;
            target_bias.dets_list.push_back(bias);    

                //}
            //}
        }
        collect = true;   
    }

    if(collect)
    {   
        target_amount = target_bias.dets_list.size();

        for(int i=0; i<target_bias.dets_list.size(); i++)
        {
            static tf::TransformBroadcaster br;
            tf::Transform tf1;
            //string tf_name1 = "tm_target_pose";
            tf1.setOrigin(tf::Vector3(target_bias.dets_list[i].x, target_bias.dets_list[i].y, target_bias.dets_list[i].z));
            tf::Quaternion q;
            q.setEuler(0.0,0.0,0.0);
            tf1.setRotation(q);
            //tf1.setRotation(tf::Quaternion(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w));
            br.sendTransform(tf::StampedTransform(tf1, ros::Time::now(), "rs_camera_link", "item_link"));
            //br.sendTransform(tf::StampedTransform(tf1, ros::Time::now(), "rs_camera_link", ""));
            string b;
            b = target_bias.dets_list[i].class_name;
            // cout << "------------------" << endl;
            // cout << target_bias.dets_list[i].x << endl;
            // cout << target_bias.dets_list[i].y << endl;
            // cout << target_bias.dets_list[i].z << endl;
            prod_list.strings.push_back(b);
        }
    }
    
    // if(collect)
    // {   
    //     target_amount = target_bias.dets_list.size();

    //     // for(int i=0; i<target_bias.dets_list.size(); i++)
    //     // {
    //     //     cout<<"test000000"<<endl;
    //     //     static tf::TransformBroadcaster br;
    //     //     tf::Transform tf1;
    //     //     string tf_name1 = "tm_target_pose";
    //     //     tf1.setOrigin(tf::Vector3(target_bias.dets_list[i].x, target_bias.dets_list[i].y, target_bias.dets_list[i].z));
    //     //     tf1.setRotation(tf::Quaternion(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w));
    //     //     br.sendTransform(tf::StampedTransform(tf1, ros::Time::now(), "camera1_link", tf_name1));

    //     //     string b;
    //     //     b = target_bias.dets_list[i].class_name;

    //     //     prod_list.strings.push_back(b);
        
    //     //     cout << target_bias.dets_list[i].x << endl;
    //     //     cout << target_bias.dets_list[i].y << endl;
    //     //     cout << target_bias.dets_list[i].z << endl;
    //     //     cout <<"-----------------------------------" <<endl;
    //     //     cout << current_pose.orientation.x << endl;
    //     //     cout << current_pose.orientation.y << endl;
    //     //     cout << current_pose.orientation.z << endl;
    //     //     cout << current_pose.orientation.w << endl;



    //     // }

    // }
}






// void nasal_swab::mis_callback(detection_msgs::StringArray msg)
// {
//     mis_list.strings = msg.strings;
//     // cout<<"0000"<<endl;
//     // cout<<"SIZE:"<<mis_list.strings.size()<<endl;
//     for(int i=0; i<msg.strings.size(); i++)
//     {
//         cout<<msg.strings[i]<<endl;
//     }
// }



// void nasal_swab::loc_callback(std_msgs::Int8 msg)
// {
//     printf("%d\n", msg.data);
//     if(msg.data == 1)
//         cout<<"At Position One"<<endl;
// }



void nasal_swab::Position_Manager()
{  
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group.setStartState(*move_group.getCurrentState());
    //move_group.setMaxVelocityScalingFactor(0.05);

    moveit_msgs::Constraints endEffector_constraints;
    moveit_msgs::OrientationConstraint ocm;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success;
    
    while(1)
    {
        command = getchar();

        if(command == 'l')
        {
            ROS_INFO("GO HOME");
            joint_group_positions = wait_for_swab;
            //move_group.setMaxVelocityScalingFactor(0.1);
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            grip.data = true;
            gripper_pub.publish(grip);
            ROS_INFO("DONE");
        }
        
        if(command == 'p')//0109測試home
        {
            ROS_INFO("REACH HOME");
            joint_group_positions = new_home_point;
            move_group.setMaxVelocityScalingFactor(0.1);
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            grip.data = true;
            gripper_pub.publish(grip);
            current_pose = move_group.getCurrentPose().pose;
            //sleep(1);
            //reach = true;
            ROS_INFO("DONE");
        }

        if(command == 'h')
        {
            ROS_INFO("REACH HOME");
            joint_group_positions = home_p;
            move_group.setMaxVelocityScalingFactor(0.1);
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            grip.data = true;
            gripper_pub.publish(grip);
            current_pose = move_group.getCurrentPose().pose;
            //sleep(1);
            //reach = true;
            ROS_INFO("DONE");
        }

        if(command == 'g')
        {
            ROS_INFO("REACH HOME");
            joint_group_positions = home_gs;
            move_group.setMaxVelocityScalingFactor(0.3);
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            grip.data = true;
            gripper_pub.publish(grip);
            current_pose = move_group.getCurrentPose().pose;
            //sleep(1);
            //reach = true;
            ROS_INFO("DONE");
        }



        if(command == 'w')
        {
            ROS_INFO("WAIT FOR SWAB");
            // // joint_group_positions = home_test;
            // // //move_group.setMaxVelocityScalingFactor(0.1);
            // // move_group.setJointValueTarget(joint_group_positions);
            // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
            // joint_group_positions=move_group.getCurrentJointValues();
            // joint_group_positions[3] -= 0.1792452335357666;
            // move_group.setJointValueTarget(joint_group_positions);
            //move_group.move();
            // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            grip.data = true;
            gripper_pub.publish(grip);
            current_pose = move_group.getCurrentPose().pose;
            sleep(1);
            //reach = true;
            ROS_INFO("DONE");
        }


        if(command == 'n')
        {
            ROS_INFO("READY TO ENTER THE NOSTRILS");
            joint_group_positions=move_group.getCurrentJointValues();
            

            //group.setStartState(*group.getCurrentState());
            moveit_msgs::Constraints endEffector_constraints;
            moveit_msgs::OrientationConstraint ocm;
            ocm.link_name = "tm_wrist_3_joint";//需要约束的链接
            ocm.header.frame_id = "base_link";//基坐标系
            
            //四元数约束
            ocm.orientation.w = 1.0;
   //欧拉角约束
            ocm.absolute_x_axis_tolerance = 0.1;
            ocm.absolute_y_axis_tolerance = 0.1;
            ocm.absolute_z_axis_tolerance = 2*3.14;
            ocm.weight = 1.0;//此限制权重
            endEffector_constraints.orientation_constraints.push_back(ocm);//加入限制列表
            move_group.setPathConstraints(endEffector_constraints);//设置约束

            
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
            //joint_group_positions = getJointValueTarget();
            //joint_group_positions.tm_elbow_1_joint +=0.5;
            //move_group.setMaxVelocityScalingFactor(0.1);
            joint_group_positions[2] -= 0.3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            grip.data = true;
            gripper_pub.publish(grip);
            //current_pose = move_group.getCurrentPose().pose;
            sleep(1);

            ROS_INFO("DONE");
        }

        if(command == 'm')
        {
            ROS_INFO("READY TO ENTER THE NOSTRILS");
            
            //group.setStartState(*group.getCurrentState());

            
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
            joint_group_positions=move_group.getCurrentJointValues();
            joint_group_positions[0] -= (0.14005903899669647-0.1224910095334053);
            joint_group_positions[1] += (0.22351965308189392-0.055379122495651245);
            joint_group_positions[2] -= (1.4275147914886475-1.2309569120407104);
            joint_group_positions[3] += (1.3428757190704346-1.3174326419830322);
            joint_group_positions[4] += (1.4828609228134155-1.463274359703064);
            joint_group_positions[5] -= (0.026606574654579163-0.023920705541968346);
            sleep(1);

//0.14005903899669647, 0.055379122495651245, 1.4275147914886475, -1.3428757190704346, 1.463274359703064, 0.026606574654579163
//0.1224910095334053, 0.22351965308189392, 1.2309569120407104, -1.3174326419830322, 1.4828609228134155, 0.023920705541968346

            //joint_group_positions[3] += 0.161913;//0.315526;
            move_group.setMaxVelocityScalingFactor(0.01);
            move_group.setJointValueTarget(joint_group_positions);
            //move_group.move();
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            grip.data = true;
            gripper_pub.publish(grip);
            //current_pose = move_group.getCurrentPose().pose;
            sleep(1);
            ROS_INFO("DONE");
        }


        if(command == 'v')
        {
            ROS_INFO("READY TO ENTER THE NOSTRILS");
            
            //group.setStartState(*group.getCurrentState());

            
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
            joint_group_positions=move_group.getCurrentJointValues();
            // joint_group_positions[0] -= (0.14005903899669647-0.1224910095334053);
            // joint_group_positions[1] += (0.22351965308189392-0.055379122495651245);
            // joint_group_positions[2] -= (1.4275147914886475-1.2309569120407104);
            // joint_group_positions[3] += (1.3428757190704346-1.3174326419830322);
            // joint_group_positions[4] += (1.4828609228134155-1.463274359703064);
            // joint_group_positions[5] -= (0.026606574654579163-0.023920705541968346);
            joint_group_positions[5] = -2.678682804107666;
          

//0.14005903899669647, 0.055379122495651245, 1.4275147914886475, -1.3428757190704346, 1.463274359703064, 0.026606574654579163
//0.1224910095334053, 0.22351965308189392, 1.2309569120407104, -1.3174326419830322, 1.4828609228134155, 0.023920705541968346

            //joint_group_positions[3] += 0.161913;//0.315526;
            move_group.setMaxVelocityScalingFactor(0.05);
            move_group.setJointValueTarget(joint_group_positions);
            //move_group.move();
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            grip.data = true;
            gripper_pub.publish(grip);
            //current_pose = move_group.getCurrentPose().pose;
            ROS_INFO("DONE");
        }

        if(command == 'b')
        {
            ROS_INFO("READY TO ENTER THE NOSTRILS");
            
            //group.setStartState(*group.getCurrentState());

            
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
            joint_group_positions=move_group.getCurrentJointValues();
            // joint_group_positions[0] -= (0.14005903899669647-0.1224910095334053);
            // joint_group_positions[1] += (0.22351965308189392-0.055379122495651245);
            // joint_group_positions[2] -= (1.4275147914886475-1.2309569120407104);
            // joint_group_positions[3] += (1.3428757190704346-1.3174326419830322);
            // joint_group_positions[4] += (1.4828609228134155-1.463274359703064);
            // joint_group_positions[5] -= (0.026606574654579163-0.023920705541968346);
            joint_group_positions[5] = (4.848136813961901e-05);
          

//0.14005903899669647, 0.055379122495651245, 1.4275147914886475, -1.3428757190704346, 1.463274359703064, 0.026606574654579163
//0.1224910095334053, 0.22351965308189392, 1.2309569120407104, -1.3174326419830322, 1.4828609228134155, 0.023920705541968346

            //joint_group_positions[3] += 0.161913;//0.315526;
            move_group.setMaxVelocityScalingFactor(0.05);
            move_group.setJointValueTarget(joint_group_positions);
            //move_group.move();
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            grip.data = true;
            gripper_pub.publish(grip);
            //current_pose = move_group.getCurrentPose().pose;
            ROS_INFO("DONE");
        }




        if(command == 'k')
        {
            ROS_INFO("test");
            cout << "x: "<< item_position.position.x<<endl;
            cout << "y: "<< item_position.position.y<<endl;
            cout << "z: "<< item_position.position.z<<endl;
            cout << "-----------------------"<< endl;
            target_pose = current_pose;
            target_pose.position.x += (item_position.position.x-0.285);
            target_pose.position.y += (item_position.position.y + 0.04);
            target_pose.position.z += (item_position.position.z+0.1);
            //target_pose.position.w=0.586061478456;
            move_group.setMaxVelocityScalingFactor(0.02);
            move_group.setPoseTarget(target_pose);
            move_group.move();
    
            ROS_INFO("DONE");
        }

       if(command == 'x')
        {
            cout << "x: "<< item_position.position.x<<endl;
            cout << "y: "<< item_position.position.y<<endl;
            cout << "z: "<< item_position.position.z<<endl;
            cout << "-----------------------"<< endl;

        }
        

        

        if(command == 't')
        {   
            if(target_amount > 0)
            {
                for(int i=0; i<target_amount; i++)
                {
                    ROS_INFO("GO TARGET POINT");
                    
                    current_pose = move_group.getCurrentPose().pose;
                    // tf::TransformListener listener;
                    // tf::StampedTransform tf2;
                    // string tf_name2 = "/tm_target_pose";
                    // listener.waitForTransform("/tm_end_eff", tf_name2 + to_string(i), ros::Time(0), ros::Duration(4.0));
                    // listener.lookupTransform("/tm_end_eff", tf_name2 + to_string(i), ros::Time(0), tf2);
            // listener.waitForTransform("/tm_end_eff", "/tm_target_pose", ros::Time(0), ros::Duration(4.0));
            // listener.lookupTransform("/tm_end_eff", "/tm_target_pose", ros::Time(0), tf2);
                    // current_pose.position.x += tf2.getOrigin().getX();
                    // current_pose.position.y += tf2.getOrigin().getZ();
                    // current_pose.position.z += tf2.getOrigin().getY();

                    // current_pose.position.x += (target_bias.dets_list[i].x-0.325);//-0.155
                    // current_pose.position.y += (target_bias.dets_list[i].y + 0.03);//+0.08
                    // current_pose.position.z += (-target_bias.dets_list[i].z-0.07);
                    current_pose.position.x = 0.722814;
                    current_pose.position.y = -0.116816;//+0.08
                    current_pose.position.z = 0.82388;          


                    cout << target_bias.dets_list[i].x << endl;
                    cout << target_bias.dets_list[i].y << endl;
                    cout << target_bias.dets_list[i].z << endl;
                    cout << "-------------" << endl;
                    
                    cout << current_pose.position.x << endl;
                    cout << current_pose.position.y << endl;
                    cout << current_pose.position.z << endl;

                    move_group.setPoseTarget(current_pose);
                    move_group.move();

                    ROS_INFO("REACH TARGET POINT");
                }
            }        
        }




        if(command == 'q')
        {
            ROS_INFO("QUIT");
            break;
        }
    }



}

void targetPoseCallback(const geometry_msgs::Pose msg)
{
    item_position = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nasal_swab");
    ros::NodeHandle nh;
    sub_item_position = nh.subscribe("/target_pose", 1000, targetPoseCallback);
    nasal_swab node(nh);
    ros::spin();
    return 0;
}




//0.14005903899669647, 0.055379122495651245, 1.4275147914886475, -1.3428757190704346, 1.463274359703064, 0.026606574654579163
//0.1224910095334053, 0.22351965308189392, 1.2309569120407104, -1.3174326419830322, 1.4828609228134155, 0.023920705541968346
//4.848136813961901e-05      -2.678682804107666