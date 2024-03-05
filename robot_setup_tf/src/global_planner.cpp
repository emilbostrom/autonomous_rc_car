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
#include <random>
#include <algorithm> 
#include <stdlib.h>

#define PI 3.14159265


std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

Quaternion ToQuaternion(double roll, double pitch, double yaw) 
{ // roll (x), pitch (y), yaw (z), angles are in radians
    
    // Abbreviations for the various angular functions
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

class Node{
    public:
        int id;
        int idParent;
        int cost;
        double stepLength;

        double headingAngle;

        double xPos;
        double yPos;
        
        Node(double xPos, double yPos, int id, double stepLength) : 
             xPos(xPos), yPos(yPos), id(id), stepLength(stepLength){
            ROS_INFO_STREAM("Node id: " << this->id);
        }

        void calcNewNodePos(Node nearestNode, double distance) {
            ROS_INFO_STREAM("Distance to node for connection: " << distance);
            double dx = static_cast<double>(xPos - nearestNode.xPos) / distance;
            double dy = static_cast<double>(yPos - nearestNode.yPos) / distance;
            this->xPos = nearestNode.xPos + dx * stepLength;
            this->yPos = nearestNode.yPos + dy * stepLength;
            ROS_INFO_STREAM("new xPos: " << this->xPos);
            ROS_INFO_STREAM("new yPos: " << this->yPos);
        }

        double calcDistance(double x1, double y1, double x2, double y2) {
            return std::max(sqrt(pow((x2-x1),2) + pow((y2-y1),2)),0.001);
        }

        Node FindNearestNode(std::vector<Node> Tree) {
            Node nearestNode = Tree[0];

            ROS_INFO_STREAM("Node " << id << " has position " << xPos << "," << yPos);

            double nearestDist = calcDistance(xPos,yPos,Tree[0].xPos,Tree[0].yPos);

            ROS_INFO_STREAM("Tree size: " << Tree.size());

            for(int i = 1; i < Tree.size(); i++) {
                double dist = calcDistance(xPos,yPos,Tree[i].xPos,Tree[i].yPos);
                if (dist < nearestDist) {
                    nearestNode = Tree[i];
                    nearestDist = dist;
                    //ROS_INFO_STREAM("New nearest node id: " << nearestNode.id);
                    //ROS_INFO_STREAM("The distance to above is: " << nearestDist);
                }
            }

            ROS_INFO_STREAM("Nearest node to " << id << " is " << nearestNode.id 
                            << "with distance: " << nearestDist);

            this->idParent = nearestNode.id;

            calcNewNodePos(nearestNode,nearestDist);

            return nearestNode;
        }

};


class GlobalPlanner{
    public:
        std::string frameIdMap = "map";
        double mapResolution; // [m/cell]
        int mapWidth; // [cells]
        int mapHeight; // [cells]
        //std::vector<int8_t> mapDataFetch; // [0-100] occupancy
        std::vector<int> mapData;
        double stepLength; // [m]
        double goalDistThreshold; // [m]
        double obstacleDistThreshold; // [m]
        
        double maxAngleDiff = PI/2; // [rad] TUNING PARAMETER!!!

        double xCurrent;
        double yCurrent;
        double zCurrent;
        double xQuat;
        double yQuat;
        double zQuat;
        double wQuat;

        double xGoal;
        double yGoal;
        double z_goal;
        double x_quat_goal;
        double y_quat_goal;
        double z_quat_goal;
        double w_quat_goal;
        EulerAngles goalEuler;

        typedef std::mt19937 MyRNG;
        MyRNG rng;

        std::uniform_int_distribution<uint32_t> widthGenerator;
        std::uniform_int_distribution<uint32_t> heightGenerator;

        std::uniform_int_distribution<uint32_t> nodeIsGoalBias = std::uniform_int_distribution<uint32_t>(0, 99);
        int goalBias = 10; // Tuning parameter

        GlobalPlanner(const geometry_msgs::PoseStamped::ConstPtr& poseMsg, 
                      const nav_msgs::OccupancyGrid::ConstPtr& mapMsg,
                      const geometry_msgs::PoseStamped::ConstPtr& goalMsg) {
            
            ROS_INFO_STREAM("Received pose: " << poseMsg);
            xCurrent = poseMsg->pose.position.x;
            yCurrent = poseMsg->pose.position.y;
            zCurrent = poseMsg->pose.position.z;
            xQuat = poseMsg->pose.orientation.x;
            yQuat = poseMsg->pose.orientation.y;
            zQuat = poseMsg->pose.orientation.z;
            wQuat = poseMsg->pose.orientation.w;

            xGoal = goalMsg->pose.position.x;
            yGoal = goalMsg->pose.position.y;
            z_goal = goalMsg->pose.position.z;
            x_quat_goal = goalMsg->pose.orientation.x;
            y_quat_goal = goalMsg->pose.orientation.y;
            z_quat_goal = goalMsg->pose.orientation.z;
            w_quat_goal = goalMsg->pose.orientation.w;

            Quaternion quatGoal = {x_quat_goal, y_quat_goal, z_quat_goal, w_quat_goal};
            goalEuler = ToEulerAngles(quatGoal);

            mapResolution = mapMsg->info.resolution;
            mapWidth = mapMsg->info.width;
            mapHeight = mapMsg->info.height;
            for (const auto& value : mapMsg->data) {
                mapData.push_back(static_cast<int>(value));
            }

            /*for(int i = 0; i < mapData.size();i++){
                if (mapData[i] != -1) {
                    ROS_INFO_STREAM("index: "<< i);
                    ROS_INFO_STREAM("value: "<< mapData[i]);
                    ROS_INFO_STREAM("x: " << i % mapHeight);
                    ROS_INFO_STREAM("y: " << i / mapHeight);
                }
            }*/

            ROS_INFO_STREAM("Map resolution [m]: " << mapResolution);
            ROS_INFO_STREAM("Map width [cells]: " << mapWidth);
            ROS_INFO_STREAM("Map height [cells]: " << mapHeight);
            ROS_INFO_STREAM("Map data size: " << mapData.size());
            
            uint32_t seed_val = 100;
            rng.seed(seed_val);
            widthGenerator = std::uniform_int_distribution<uint32_t>(0, mapWidth);
            heightGenerator = std::uniform_int_distribution<uint32_t>(0, mapHeight);

            stepLength = mapResolution*2;
            goalDistThreshold = stepLength*2;
            obstacleDistThreshold = stepLength*2;
        }

        std::tuple<double, double> CellToCoordinate(int xCell, int yCell) {
            double xPos = (xCell - mapWidth/2)*mapResolution;
            double yPos = (yCell - mapHeight/2)*mapResolution;
            return {xPos, yPos};
        }

        double calcDistance(double x1, double y1, double x2, double y2) {
            return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
        }

        bool checkForObstacle(Node node) {
            int xMapCell = node.xPos / mapResolution + mapWidth/2;
            int yMapCell = node.yPos / mapResolution + mapHeight/2;

            int mapDataIndex = yMapCell*mapHeight + xMapCell;
            
            //ROS_INFO_STREAM("Map data index: " << mapDataIndex);
            //ROS_INFO_STREAM("Node position " << node.xPos << "," << node.yPos <<  " is in mapdata: " << this->mapData[mapDataIndex]);
            if (this->mapData[mapDataIndex] != 0){
                return true;
            } else {
                return false;
            }
        }

        bool checkDynamicConstraints(Node node, Node nodeParent){

            double xDelta = node.xPos - nodeParent.xPos;
            double yDelta = node.yPos - nodeParent.yPos;
            double nodeHeading = atan2((yDelta), xDelta);
            ROS_INFO_STREAM("Node heading cal: " << nodeHeading);
            double headingDiff = PI/2 - abs(abs(nodeHeading - nodeParent.headingAngle) - PI/2); 
            ROS_INFO_STREAM("Node heading diff: " << headingDiff);
            if (headingDiff < maxAngleDiff) {
                node.headingAngle = nodeHeading;
                return false;
            }
            
            return true;
        }

        nav_msgs::Path createPathToGoal(std::vector<Node> Tree) {
            
            ROS_INFO_STREAM("Goal node reached, creating path msg");

            std::vector<geometry_msgs::PoseStamped> posesStampedVectorMsg;
            
            geometry_msgs::PoseStamped poseStampedMsg;
            poseStampedMsg.header.frame_id = frameIdMap;
            
            geometry_msgs::Pose pose;
            
            nav_msgs::Path path;

            Node nodePrev = Tree.back();
            ROS_INFO_STREAM("First node id: " << nodePrev.id);
            ROS_INFO_STREAM("First node parent id: " << nodePrev.idParent);
            
            for (int i = Tree.size() - 1; i >= 0; --i) {
                Node node = Tree[i];
                ROS_INFO_STREAM("Node id: " << node.id);
                if(node.id != nodePrev.idParent) {
                    continue;
                }
                pose.position.x = node.xPos;
                pose.position.y = node.yPos;
                pose.position.z = 0;

                Quaternion quat = ToQuaternion(0, 0, node.headingAngle);
                ROS_INFO_STREAM("Quaternion: x: " << quat.x << " y: " << quat.y << " z: " 
                                 << quat.z << " w: " << quat.w);

                pose.orientation.x = quat.x;
                pose.orientation.y = quat.y;
                pose.orientation.z = quat.z;
                pose.orientation.w = quat.w;

                poseStampedMsg.pose = pose;

                posesStampedVectorMsg.push_back(poseStampedMsg);

                nodePrev = node;
            }

            path.header.frame_id = frameIdMap;
            path.poses = posesStampedVectorMsg;

            return path;
        }

        const nav_msgs::Path createPath(ros::Publisher pub){

            double xPathPos = xCurrent;
            double yPathPos = yCurrent;
            nav_msgs::Path path;

            double distToGoal = calcDistance(xPathPos,yPathPos,xGoal,yGoal);

            // Create first node, which is current position
            int idOrigin = 0;
            Node nodeOrigin(xCurrent,yCurrent,idOrigin,stepLength);
            nodeOrigin.idParent = 0;
            nodeOrigin.cost = 0;
            Quaternion quat = {xQuat, yQuat, zQuat, wQuat};
            EulerAngles angle = ToEulerAngles(quat);
            nodeOrigin.headingAngle = angle.yaw;

            ROS_INFO_STREAM("First node heading: " << nodeOrigin.headingAngle);

            // Create an array of all nodes
            std::vector<Node> Tree;
            Tree.push_back(nodeOrigin);

            int maxIterationsRrt = 2000;
            for(int iRrt  = 1; iRrt < maxIterationsRrt; iRrt++) {
                
                int xCell = widthGenerator(rng);
                int yCell = heightGenerator(rng);

                auto [xPosNode, yPosNode] = CellToCoordinate(xCell,yCell);

                if(nodeIsGoalBias(rng) < goalBias){
                    ROS_INFO_STREAM("Node is set to goal");
                    xPosNode = xGoal;
                    yPosNode = yGoal;
                }

                Node newNode(xPosNode,yPosNode,iRrt,stepLength);
                Node parentNode = newNode.FindNearestNode(Tree);

                bool dynamicConstraint = checkDynamicConstraints(newNode,parentNode);
                if (dynamicConstraint) {
                    ROS_INFO_STREAM("Node does not meet dynamic constraints, skipped");
                    continue;
                }

                bool nodeInObstacle = checkForObstacle(newNode);
                if (nodeInObstacle) {
                    ROS_INFO_STREAM("Node in obstacle, skipped");
                    continue;
                }
 
                Tree.push_back(newNode);

                ROS_INFO_STREAM("Added new node to tree: " << newNode.id << " xPos: " 
                                 << newNode.xPos << " yPos: " << newNode.yPos);
                
                distToGoal = calcDistance(newNode.xPos,newNode.yPos,xGoal,yGoal);
                
                double headingDiffToGoal = PI/2 - abs(abs(newNode.headingAngle - goalEuler.yaw) - PI/2); 
                if (distToGoal < goalDistThreshold && headingDiffToGoal < maxAngleDiff) {
                    path = createPathToGoal(Tree);
                    return path;
                }
            }
            path = createPathToGoal(Tree);
            return path;
        }

};


int main(int argc, char** argv) {

    ros::init(argc,argv, "global_planner");
    ros::NodeHandle n;
    
    boost::shared_ptr<geometry_msgs::PoseStamped const> iniPosMsg;
    iniPosMsg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("slam_out_pose",ros::Duration(2.0));

    boost::shared_ptr<nav_msgs::OccupancyGrid const> mapDataMsg;
    mapDataMsg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map",ros::Duration(2.0));

    boost::shared_ptr<geometry_msgs::PoseStamped const> goalPosMsg;
    goalPosMsg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/move_base_simple/goal",ros::Duration(30.0));

    GlobalPlanner planner(iniPosMsg,mapDataMsg,goalPosMsg);

    ros::Publisher pub = n.advertise<nav_msgs::Path>("global_path",10);

    ros::Rate r(10); // 10 hz

    nav_msgs::Path path = planner.createPath(pub);

    while(ros::ok()) {

        pub.publish(path);

        ros::spinOnce();
        r.sleep();
    }

    
    return 0;

}