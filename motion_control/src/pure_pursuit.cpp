#include <stdlib.h>

#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm> 

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class PurePursuit
{
    public:
        // Tuning parameters
        const double lookAheadDistance = 0.5; // [m]
        const double RATE = 0.1; // [s] 1/RATE = Hz

        double xCurrent, yCurrent;
        double xQuatCurrent,yQuatCurrent,zQuatCurrent,wQuatCurrent;        

        std::vector<geometry_msgs::Pose> pathPoses;

        PurePursuit():
            n{},
            subPos(n.subscribe("slam_out_pose", 1000, &PurePursuit::poseCallback, this)),
            subMap(n.subscribe("global_path", 1000, &PurePursuit::poseCallback, this)),
            timer(n.createTimer(ros::Duration(RATE), &PurePursuit::main_loop, this))
        {
        }

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& currentPosMsg) {
            xCurrent = currentPosMsg->pose.position.x;
            yCurrent = currentPosMsg->pose.position.y;
            xQuatCurrent = currentPosMsg->pose.orientation.x;
            yQuatCurrent = currentPosMsg->pose.orientation.y;
            zQuatCurrent = currentPosMsg->pose.orientation.z;
            wQuatCurrent = currentPosMsg->pose.orientation.w;
            ROS_INFO_STREAM("xCurrent: " << xCurrent);
            ROS_INFO_STREAM("yCurrent: " << yCurrent);
        }

        void pathCallback(const nav_msgs::Path::ConstPtr& pathMsg) {
            for(const auto& poseStampedMsg : pathMsg->poses) {
                pathPoses.push_back(poseMsg->pose);
                ROS_INFO_STREAM("path x : " << poseStampedMsg->pose->position->x);
            }
        }

        void findClosestPointToCar(){

        }

        void findLookAheadPoint(){

        }

        void transformPointToVehicleCoordSys(){

        }

        void calcSteeringAngle(){

        }

        void main_loop(const ros::TimerEvent &)
        {
            findClosestPointToCar();
            findLookAheadPoint();
            transformPointToVehicleCoordSys();
            calcSteeringAngle();
        }

    private:
        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Subscriber subMap;
        ros::Subscriber subPos;
        ros::Timer timer;
};

int main(int argc, char** argv){
    ros::init(argc,argv, "pure_pursuit");
    PurePursuit controller;
    ros::spin();
    return 0;
}