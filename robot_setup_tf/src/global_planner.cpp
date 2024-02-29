#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include <vector>

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;

double x_current = 0;
double y_current = 0;
double z_current = 0;
double x_quat = 0;
double y_quat = 0;
double z_quat = 0;
double w_quat = 0;

double x_goal = 5;
double y_goal = 5;
double z_goal = 0;
double x_quat_goal = 0.707;
double y_quat_goal = 0;
double z_quat_goal = 0;
double w_quat_goal = 0.707;

string frame_id_goal = "map";

void positionSubscriber(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO_STREAM("Received pose: " << msg);
    x_current = msg->pose.position.x;
    y_current = msg->pose.position.y;
    z_current = msg->pose.position.z;
    x_quat = msg->pose.orientation.x;
    y_quat = msg->pose.orientation.y;
    z_quat = msg->pose.orientation.z;
    w_quat = msg->pose.orientation.w;

    ROS_INFO_STREAM(x_current);
    ROS_INFO_STREAM(y_current);
    ROS_INFO_STREAM(z_current);
    ROS_INFO_STREAM(x_quat);
    ROS_INFO_STREAM(y_quat);
    ROS_INFO_STREAM(z_quat);
    ROS_INFO_STREAM(w_quat);
}

int main(int argc, char** argv) {

    ros::init(argc,argv, "global_planner");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("slam_out_pose",1000,positionSubscriber);
    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("goal_position",10);

    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        
        geometry_msgs::PoseStamped goal_pos_msg;
        goal_pos_msg.pose.position.x = x_goal;
        goal_pos_msg.pose.position.y = y_goal;
        goal_pos_msg.pose.position.z = z_goal;
        goal_pos_msg.pose.orientation.x = x_quat_goal;
        goal_pos_msg.pose.orientation.y = y_quat_goal;
        goal_pos_msg.pose.orientation.z = z_quat_goal;
        goal_pos_msg.pose.orientation.w = w_quat_goal;
        
        geometry_msgs.header.frame_id = frame_id_goal;


        pub.publish(goal_pos_msg);

        ros::spinOnce();
        r.sleep();
    }

    
    return 0;

}