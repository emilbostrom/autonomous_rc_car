#include <stdlib.h>

#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstring>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <string.h>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Serial port
int serial_port = open("/dev/ttyUSB1", O_RDWR);

// Tuning constants
int MAX_STEERING_ANGLE = 1; // [rad]

double calcDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
}

class PurePursuit
{
    public:
        // Tuning parameters
        const double lookAheadDistance = 0.5; // [m]
        const double RATE = 0.1; // [s] 1/RATE = Hz

        double xCurrent, yCurrent;
        double xQuatCurrent, yQuatCurrent, zQuatCurrent, wQuatCurrent;
        double currentHeading, dummyRoll, dummyPitch;   

        int closestPoint, lookAheadPoint;
        double deltaY;
        double steeringAngle, steeringCommand;

        std::vector<geometry_msgs::Pose> pathPoses;

        PurePursuit():
            n{},
            subPos(n.subscribe("slam_out_pose", 1000, &PurePursuit::poseCallback, this)),
            subPath(n.subscribe("global_path", 1000, &PurePursuit::pathCallback, this)),
            timer(n.createTimer(ros::Duration(RATE), &PurePursuit::main_loop, this))
        {
            ROS_INFO_STREAM("Initialize pure pursuit");
        }

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& currentPosMsg) {
            xCurrent = currentPosMsg->pose.position.x;
            yCurrent = currentPosMsg->pose.position.y;
            xQuatCurrent = currentPosMsg->pose.orientation.x;
            yQuatCurrent = currentPosMsg->pose.orientation.y;
            zQuatCurrent = currentPosMsg->pose.orientation.z;
            wQuatCurrent = currentPosMsg->pose.orientation.w;
            tf2::Quaternion q(currentPosMsg->pose.orientation.x, 
                            currentPosMsg->pose.orientation.y, 
                            currentPosMsg->pose.orientation.z, 
                            currentPosMsg->pose.orientation.w
            );
            tf2::Matrix3x3 m(q);
            m.getRPY(dummyRoll,dummyPitch,currentHeading);
            ROS_INFO_STREAM("xCurrent: " << xCurrent);
            ROS_INFO_STREAM("yCurrent: " << yCurrent);
        }

        void pathCallback(const nav_msgs::Path::ConstPtr& pathMsg) {
            ROS_INFO_STREAM("pathMsg" << pathMsg);
            for(const auto& poseStampedMsg : pathMsg->poses) {
                pathPoses.push_back(poseStampedMsg.pose);
            }
        }

        void findClosestPointToCar(){
            double dist;
            double closestDist = 1000000; // Large starting value
            ROS_INFO_STREAM("Path size: " << pathPoses.size());
            for(int i = 0; i < pathPoses.size(); i++) {
                dist = calcDistance(pathPoses[i].position.x,pathPoses[i].position.y,
                                    xCurrent, yCurrent
                );
                ROS_INFO_STREAM("Dist to point " << i << " on path is: " << dist);
                if (dist < closestDist){
                    closestPoint = i;
                    closestDist = dist;
                }
            }
        }

        void findLookAheadPoint(){
            double dist;
            double closestDist = 1000000; // Large starting value
            for(int i = 0; i < pathPoses.size(); i++) {
                dist = calcDistance(pathPoses[i].position.x,pathPoses[i].position.y, 
                                    pathPoses[closestPoint].position.x,pathPoses[closestPoint].position.y);
                ROS_INFO_STREAM("Dist to point " << i << " from closest point " << closestPoint << " is: " << dist);
                if (abs(dist-lookAheadDistance) < closestDist){
                    lookAheadPoint = i;
                    closestDist = abs(dist-lookAheadDistance);
                }
            }
        }

        void transformPointToVehicleCoordSys(){
            // y'= -*sin(theta)*x + cos(theta)*y
            deltaY = -sin(currentHeading)*pathPoses[lookAheadPoint].position.x + 
                    cos(currentHeading)*pathPoses[lookAheadPoint].position.y;
        }

        void setSteeringCommand(){
            steeringCommand = steeringAngle / MAX_STEERING_ANGLE;
            if (steeringCommand > 1.0)
                steeringCommand = 1.0;
            else if (steeringCommand < -1.0)
                steeringCommand = -1.0;
        }

        void calcSteeringAngle(){
            steeringAngle = 2*deltaY/pow(lookAheadDistance,2);
            setSteeringCommand();
        }

        void sendsteeringCommand(){
            std::string steeringString = std::to_string(steeringCommand);
            std::string steeringMessage = "S" + steeringString;
            const char* steeringMessagePtr = steeringMessage.c_str();
            write(serial_port, steeringMessagePtr, strlen(steeringMessagePtr));
            ROS_INFO_STREAM("Message sent: " << steeringMessage);
        }

        void main_loop(const ros::TimerEvent &)
        {
            ROS_INFO_STREAM("A loop");
            if (pathPoses.size() != 0){
                findClosestPointToCar();
                findLookAheadPoint();
                transformPointToVehicleCoordSys();
                calcSteeringAngle();
                sendsteeringCommand();
            }
            
        }

    private:
        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Subscriber subPath;
        ros::Subscriber subPos;
        ros::Timer timer;
};

void configureSerialPort(){
    std::cout << "Configuring serial port";
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (serial_port < 0) {
        std::cout << "Error from open";
    }
    if(tcgetattr(serial_port, &tty) != 0) {
        std::cout << "Error from tcgetattr";
    }
    cfsetospeed(&tty, B115200);
}

int main(int argc, char** argv){
    configureSerialPort();
    ros::init(argc,argv, "pure_pursuit");
    PurePursuit controller;
    ros::spin();
    close(serial_port);
    return 0;
}