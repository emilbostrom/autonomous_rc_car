#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>


double x_goal = 5;
double y_goal = 5;
double z_goal = 0;
double x_quat_goal = 0.707;
double y_quat_goal = 0;
double z_quat_goal = 0;
double w_quat_goal = 0.707;



std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;

class GlobalPlanner{
    public:
        std::string frame_id_map = "map";
        
        double step_length = 0.1;

        double goal_dist_threshold = step_length*2;

        double x_current = 0;
        double y_current = 0;
        double z_current = 0;
        double x_quat = 0;
        double y_quat = 0;
        double z_quat = 0;
        double w_quat = 0;


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

        double CalcDistance(double x1, double y1, double x2, double y2) {
            return sqrt(pow((x2-x1),2) + pow((y2-y1),2))  ;
        }

        const nav_msgs::Path CreatePath(){
            
            nav_msgs::Path path;
            std::vector<geometry_msgs::PoseStamped> poses_stamped;
            poses_stamped.frame_id = frame_id_map;

            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = frame_id_map;

            geometry_msgs::Pose pose;

            double x_path_pos = x_current;
            double y_path_pos = y_current;

            double dist_to_goal = CalcDistance(x_path_pos,y_path_pos,x_goal,y_goal);

            while(dist_to_goal > goal_dist_threshold) {
                x_path_pos += step_length;
                y_path_pos += step_length;
                
                pose.position.x = x_path_pos;
                pose.position.y = y_path_pos;
                pose.position.z = 0;

                pose.orientation.x = 0.924;
                pose.orientation.y = 0;
                pose.orientation.z = 0;
                pose.orientation.w = 0.383;

                pose_stamped.pose = pose;

                poses_stamped.push_back(pose_stamped);
                
                dist_to_goal = CalcDistance(x_path_pos,y_path_pos,x_goal,y_goal);
            }

            path.header.frame_id = frame_id_map;
            path.poses = poses_stamped;
            return path;
        }

};


int main(int argc, char** argv) {

    GlobalPlanner planner;

    ros::init(argc,argv, "global_planner");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("slam_out_pose",1000, &GlobalPlanner::positionSubscriber, &planner);
    ros::Publisher pub = n.advertise<nav_msgs::Path>("global_path",10);

    ros::Rate r(10); // 10 hz
    

    geometry_msgs::PoseStamped goal_pos_msg;

    goal_pos_msg.pose.position.x = x_goal;
    goal_pos_msg.pose.position.y = y_goal;
    goal_pos_msg.pose.position.z = z_goal;
    goal_pos_msg.pose.orientation.x = x_quat_goal;
    goal_pos_msg.pose.orientation.y = y_quat_goal;
    goal_pos_msg.pose.orientation.z = z_quat_goal;
    goal_pos_msg.pose.orientation.w = w_quat_goal;
    
    goal_pos_msg.header.frame_id = planner.frame_id_map;

    nav_msgs::Path path = planner.CreatePath();

    pub.publish(path);

    ros::spinOnce();

    
    return 0;

}