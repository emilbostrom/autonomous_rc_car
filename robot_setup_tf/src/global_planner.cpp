#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>


double xGoal = 5;
double yGoal = 5;
double z_goal = 0;
double x_quat_goal = 0.707;
double y_quat_goal = 0;
double z_quat_goal = 0;
double w_quat_goal = 0.707;



std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;

class GlobalPlanner{
    public:
        std::string frameIdMap = "map";
        double stepLength = 0.1;
        double goalDistThreshold = stepLength*2;

        double xCurrent;
        double yCurrent;
        double zCurrent;
        double xQuat;
        double yQuat;
        double zQuat;
        double wQuat;

        void initialPosition(const geometry_msgs::PoseStamped::ConstPtr& msg) {
            ROS_INFO_STREAM("Received pose: " << msg);
            xCurrent = msg->pose.position.x;
            yCurrent = msg->pose.position.y;
            zCurrent = msg->pose.position.z;
            xQuat = msg->pose.orientation.x;
            yQuat = msg->pose.orientation.y;
            zQuat = msg->pose.orientation.z;
            wQuat = msg->pose.orientation.w;

            ROS_INFO_STREAM(xCurrent);
            ROS_INFO_STREAM(yCurrent);
            ROS_INFO_STREAM(zCurrent);
            ROS_INFO_STREAM(xQuat);
            ROS_INFO_STREAM(yQuat);
            ROS_INFO_STREAM(zQuat);
            ROS_INFO_STREAM(wQuat);
        }

        double calcDistance(double x1, double y1, double x2, double y2) {
            return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
        }

        const nav_msgs::Path createPath(){
            
            nav_msgs::Path path;
            std::vector<geometry_msgs::PoseStamped> posesStampedVectorMsg;

            geometry_msgs::PoseStamped poseStampedMsg;
            poseStampedMsg.header.frame_id = frameIdMap;

            geometry_msgs::Pose pose;

            double xPathPos = xCurrent;
            double yPathPos = yCurrent;

            double distToGoal = calcDistance(xPathPos,yPathPos,xGoal,yGoal);

            while(distToGoal > goalDistThreshold) {
                xPathPos += stepLength;
                yPathPos += stepLength;
                
                pose.position.x = xPathPos;
                pose.position.y = yPathPos;
                pose.position.z = 0;

                pose.orientation.x = 0.924;
                pose.orientation.y = 0;
                pose.orientation.z = 0;
                pose.orientation.w = 0.383;

                poseStampedMsg.pose = pose;

                posesStampedVectorMsg.push_back(poseStampedMsg);
                
                distToGoal = calcDistance(xPathPos,yPathPos,xGoal,yGoal);
            }

            path.header.frame_id = frameIdMap;
            path.poses = posesStampedVectorMsg;
            return path;
        }

};


int main(int argc, char** argv) {

    GlobalPlanner planner;

    ros::init(argc,argv, "global_planner");
    ros::NodeHandle n;
    
    boost::shared_ptr<geometry_msgs::PoseStamped const> iniPosMsg;
    iniPosMsg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("slam_out_pose",ros::Duration(10.0));

    planner.initialPosition(iniPosMsg);

    //ros::Subscriber sub = n.subscribe("slam_out_pose",1000, &GlobalPlanner::positionSubscriber, &planner);
    ros::Publisher pub = n.advertise<nav_msgs::Path>("global_path",10);

    ros::Rate r(10); // 10 hz

    nav_msgs::Path path = planner.createPath();

    while(ros::ok()) {

        pub.publish(path);

        ros::spinOnce();
        r.sleep();
    }

    
    return 0;

}