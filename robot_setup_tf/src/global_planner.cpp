#include <stdlib.h>

#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <random>
#include <algorithm> 

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// TUNING PARAMETERS
const double maxAngleDiff = 0.5;// M_PI/8; 
const int goalBias = 1; // How often should the new node be set in goal
const int maxIterationsRrt = 2000;
const double stepLength = 0.1; // [m]
const double goalDistThreshold = 3*stepLength; // [m]
const double obstacleDistThreshold = stepLength; // [m]

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

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

double calcDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
}

double calcHeadingDiff(double heading1, double heading2) {
    double headingDiff = heading1 - heading2;
    
    while (headingDiff > M_PI) {
        headingDiff -= 2.0 * M_PI;
    } 
    while (headingDiff <= -M_PI) {
        headingDiff += 2.0 * M_PI;
    }
    ROS_INFO_STREAM("Node heading diff: " << headingDiff);
    return headingDiff;
}

class Node{
    public:
        int id;
        int idParent;
        int cost;

        double headingAngle;

        double xPos;
        double yPos;
        
        Node(double xPos, double yPos, int id) : 
            xPos(xPos), yPos(yPos), id(id){
            ROS_INFO_STREAM("Node id: " << id);
        }

        void calcNewNodePos(const Node& nearestNode, double distance) {
            ROS_INFO_STREAM("Distance to node for connection: " << distance);
            double dx = static_cast<double>(xPos - nearestNode.xPos) / distance;
            double dy = static_cast<double>(yPos - nearestNode.yPos) / distance;

            if (distance > stepLength) {
                dx = dx*stepLength;
                dy = dy*stepLength;
            }

            std::tie(dx,dy) = checkDynamicConstraints(dx,dy,nearestNode);

            xPos = nearestNode.xPos + dx;
            yPos = nearestNode.yPos + dy;
            ROS_INFO_STREAM("stepLength: " << stepLength);
            ROS_INFO_STREAM("dx: " << dx);
            ROS_INFO_STREAM("dy: " << dy);
            ROS_INFO_STREAM("new xPos: " << xPos);
            ROS_INFO_STREAM("new yPos: " << yPos);
        }

        std::tuple<double, double> checkDynamicConstraints(double dx, double dy, const Node& nodeParent){

            headingAngle = atan2(dy,dx);
            
            ROS_INFO_STREAM("Node heading calc: " << headingAngle);
            double headingDiff = calcHeadingDiff(headingAngle,nodeParent.headingAngle);
        
            if (abs(headingDiff) > maxAngleDiff) {
                ROS_INFO_STREAM("Heading max:");
                ROS_INFO_STREAM("Previous heading: " << headingAngle);
                int headingSign = (headingDiff > 0) - (headingDiff < 0);
                headingAngle = headingSign*maxAngleDiff + nodeParent.headingAngle;
                while (headingAngle > M_PI) {
                    headingAngle -= 2.0 * M_PI;
                } 
                while (headingAngle <= -M_PI) {
                    headingAngle += 2.0 * M_PI;
                }
                ROS_INFO_STREAM("New heading: " << headingAngle);
                ROS_INFO_STREAM("Prev dx: " << dx);
                ROS_INFO_STREAM("Prev dy: " << dy);
                dx = cos(headingAngle)*stepLength;
                dy = sin(headingAngle)*stepLength;
                ROS_INFO_STREAM("New dx: " << dx);
                ROS_INFO_STREAM("New dy: " << dy);
            }
            
            return {dx,dy};
        }

        /*bool checkDynamicConstraints(const Node& nodeParent){

            double xDelta = xPos - nodeParent.xPos;
            double yDelta = yPos - nodeParent.yPos;
            double nodeHeading = atan2(yDelta,xDelta);
            
            ROS_INFO_STREAM("Node heading calc: " << nodeHeading);
            double headingDiff = calcHeadingDiff(nodeHeading,nodeParent.headingAngle);
        
            if (abs(headingDiff) < maxAngleDiff) {
                headingAngle = nodeHeading;
                return false;
            }
            
            return true;
        }*/

        Node findNearestNode(const std::vector<Node>& Tree) {
            Node nearestNode = Tree[0];

            ROS_INFO_STREAM("Node " << id << " has position " << xPos << "," << yPos);

            double nearestDist = calcDistance(xPos,yPos,Tree[0].xPos,Tree[0].yPos);

            ROS_INFO_STREAM("Tree size: " << Tree.size());

            for(int i = 1; i < Tree.size(); i++) {
                double dist = calcDistance(xPos,yPos,Tree[i].xPos,Tree[i].yPos);
                if (dist < nearestDist) {
                    nearestNode = Tree[i];
                    nearestDist = dist;
                }
            }

            ROS_INFO_STREAM("Nearest node to " << id << " is " << nearestNode.id 
                            << "with distance: " << nearestDist);

            idParent = nearestNode.id;

            calcNewNodePos(nearestNode,nearestDist);

            return nearestNode;
        }

};


class GlobalPlanner{
    public:
        
        // Map variables
        std::string frameIdMap = "map";
        double mapResolution; // [m/cell]
        int mapWidth; // [cells]
        int mapHeight; // [cells]
        std::vector<int> mapData;

        // Current pose variables
        double xCurrent;
        double yCurrent;
        double zCurrent;
        double xQuatCurrent;
        double yQuatCurrent;
        double zQuatCurrent;
        double wQuatCurrent;

        // Goal variables
        double xGoal;
        double yGoal;
        double z_goal;
        double x_quat_goal;
        double y_quat_goal;
        double z_quat_goal;
        double w_quat_goal;
        double goalHeading;

        // Random generator
        std::mt19937 rng;
        uint32_t seed_val;

        GlobalPlanner(const geometry_msgs::PoseStamped::ConstPtr& poseMsg, 
                      const nav_msgs::OccupancyGrid::ConstPtr& mapMsg,
                      const geometry_msgs::PoseStamped::ConstPtr& goalMsg) {
            
            ROS_INFO_STREAM("Received pose: " << poseMsg);
            xCurrent = poseMsg->pose.position.x;
            yCurrent = poseMsg->pose.position.y;
            zCurrent = poseMsg->pose.position.z;
            xQuatCurrent = poseMsg->pose.orientation.x;
            yQuatCurrent = poseMsg->pose.orientation.y;
            zQuatCurrent = poseMsg->pose.orientation.z;
            wQuatCurrent = poseMsg->pose.orientation.w;

            ROS_INFO_STREAM("xCurrent: " << xCurrent);
            ROS_INFO_STREAM("yCurrent: " << yCurrent);
            ROS_INFO_STREAM("zCurrent: " << zCurrent);
            ROS_INFO_STREAM("xQuatCurrent: " << xQuatCurrent);
            ROS_INFO_STREAM("yQuatCurrent: " << yQuatCurrent);
            ROS_INFO_STREAM("zQuatCurrent: " << zQuatCurrent);
            ROS_INFO_STREAM("wQuatCurrent: " << wQuatCurrent);

            xGoal = goalMsg->pose.position.x;
            yGoal = goalMsg->pose.position.y;
            z_goal = goalMsg->pose.position.z;
            x_quat_goal = goalMsg->pose.orientation.x;
            y_quat_goal = goalMsg->pose.orientation.y;
            z_quat_goal = goalMsg->pose.orientation.z;
            w_quat_goal = goalMsg->pose.orientation.w;

            //Quaternion quatGoal = {x_quat_goal, y_quat_goal, z_quat_goal, w_quat_goal};
            tf2::Quaternion q(x_quat_goal, y_quat_goal, z_quat_goal, w_quat_goal);
            tf2::Matrix3x3 m(q);
            double rollGoal, pitchGoal, yawGoal;
            m.getRPY(rollGoal, pitchGoal, yawGoal);
            goalHeading = yawGoal;
            ROS_INFO_STREAM("goalHeading:" << goalHeading);

            mapResolution = mapMsg->info.resolution;
            mapWidth = mapMsg->info.width;
            mapHeight = mapMsg->info.height;
            for (const auto& value : mapMsg->data) {
                mapData.push_back(static_cast<int>(value));
            }

            seed_val = 100;
            rng.seed(seed_val);
        }

        std::tuple<double, double> randomPos() {
            std::uniform_int_distribution<uint32_t> widthGenerator(0, mapWidth);
            std::uniform_int_distribution<uint32_t> heightGenerator(0, mapHeight);


            int xCell = widthGenerator(rng);
            int yCell = heightGenerator(rng);
            
            double xPos = (xCell - mapWidth/2)*mapResolution;
            double yPos = (yCell - mapHeight/2)*mapResolution;

            return {xPos, yPos};
        }

        int posToCell(double xPos, double yPos) {
            int xMapCell = xPos / mapResolution + mapWidth/2;
            int yMapCell = yPos / mapResolution + mapHeight/2;
            return yMapCell*mapHeight + xMapCell; 
        }

        bool checkForObstacle(const Node& node) {
            // Create an area around the point that is also forbidden
            for (double x = node.xPos - obstacleDistThreshold; x < node.xPos + obstacleDistThreshold; x += mapResolution){
                for (double y = node.yPos - obstacleDistThreshold; y < node.yPos + obstacleDistThreshold; y += mapResolution){
                    if (mapData[posToCell(x,y)] != 0){
                        ROS_INFO_STREAM("Obstacle cell found");
                        return true;
                    }
                }
            }
            return false;
        }

        void findNodeClosestToGoal(const Node& nodePrev, const std::vector<Node>& Tree) {
            ROS_INFO_STREAM("Goal node not found, creating path to closest node");
            double closestDist = 10000.0;
            double dist;
            for (const Node& node : Tree) {
                dist = calcDistance(node.xPos,node.yPos,xGoal,yGoal);
                if (dist < closestDist) {
                    nodePrev = node;
                    closestDist = dist;
                }
            }
        }

        nav_msgs::Path createPathToGoal(const std::vector<Node>& Tree, bool goalFound) {

            std::vector<geometry_msgs::PoseStamped> posesStampedVectorMsg;
            
            geometry_msgs::PoseStamped poseStampedMsg;
            poseStampedMsg.header.frame_id = frameIdMap;
            
            geometry_msgs::Pose pose;
            
            nav_msgs::Path path;
            
            Node nodePrev =  Tree.back();
            if (goalFound != true) {
                findNodeClosestToGoal(nodePrev, Tree);
            }
            
            ROS_INFO_STREAM("First node id: " << nodePrev.id);
            ROS_INFO_STREAM("First node parent id: " << nodePrev.idParent);
            
            for (int i = Tree.size() - 1; i >= 0; --i) {
                Node node = Tree[i];
                if(node.id != nodePrev.idParent) {
                    continue;
                }
                ROS_INFO_STREAM("Node id: " << node.id);

                pose.position.x = node.xPos;
                pose.position.y = node.yPos;
                pose.position.z = 0;

                tf2::Quaternion quat;
                quat.setRPY(0,0,node.headingAngle);
                quat=quat.normalize();

                ROS_INFO_STREAM("Node heading angle: " << node.headingAngle);
                ROS_INFO_STREAM("Node parent id: " << node.idParent);

                pose.orientation.x = quat.getX();
                pose.orientation.y = quat.getY();
                pose.orientation.z = quat.getZ();
                pose.orientation.w = quat.getW();

                poseStampedMsg.pose = pose;

                posesStampedVectorMsg.push_back(poseStampedMsg);

                nodePrev = node;
            }

            path.header.frame_id = frameIdMap;
            path.poses = posesStampedVectorMsg;

            return path;
        }

        const nav_msgs::Path createPath(){

            nav_msgs::Path path;

            // Create first node, which is current position
            int idOrigin = 0;
            Node nodeOrigin(xCurrent,yCurrent,idOrigin);
            nodeOrigin.idParent = 0;
            nodeOrigin.cost = 0;
            // Quaternion quat = {xQuatCurrent, yQuatCurrent, zQuatCurrent, wQuatCurrent};
            // EulerAngles angle = ToEulerAngles(quat);
            tf2::Quaternion q(xQuatCurrent, yQuatCurrent, zQuatCurrent, wQuatCurrent);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            nodeOrigin.headingAngle = yaw;

            ROS_INFO_STREAM("roll:" << roll);
            ROS_INFO_STREAM("pitch:" << pitch);
            ROS_INFO_STREAM("First node heading: " << nodeOrigin.headingAngle);

            // Create an array of all nodes
            std::vector<Node> Tree;
            Tree.push_back(nodeOrigin);

             std::uniform_int_distribution<uint32_t> nodeIsGoalBias(0, 99);

            double xPosNode, yPosNode, distToGoal;
            for(int iRrt  = 1; iRrt < maxIterationsRrt; iRrt++) {
                // Randomly set the new node in the goal position 
                if(nodeIsGoalBias(rng) < goalBias){
                    ROS_INFO_STREAM("Node is set to goal");
                    xPosNode = xGoal;
                    yPosNode = yGoal;
                } else {
                    std::tie(xPosNode, yPosNode) = randomPos();
                }

                Node newNode(xPosNode,yPosNode,iRrt);
                Node parentNode = newNode.findNearestNode(Tree);

                /*bool dynamicConstraint = newNode.checkDynamicConstraints(parentNode);
                if (dynamicConstraint) {
                    ROS_INFO_STREAM("Node does not meet dynamic constraints, skipped");
                    continue;
                }*/

                bool nodeInObstacle = checkForObstacle(newNode);
                if (nodeInObstacle) {
                    ROS_INFO_STREAM("Node in obstacle, skipped");
                    continue;
                }
 
                // If both dynamic and obstacle checks pass add node to tree
                Tree.push_back(newNode);

                ROS_INFO_STREAM("Added new node to tree: " << newNode.id << " xPos: " 
                                 << newNode.xPos << " yPos: " << newNode.yPos);
                
                distToGoal = calcDistance(newNode.xPos,newNode.yPos,xGoal,yGoal);
                
                double headingDiffToGoal = calcHeadingDiff(newNode.headingAngle,goalHeading);
                if (distToGoal < goalDistThreshold && abs(headingDiffToGoal) < maxAngleDiff) {
                    path = createPathToGoal(Tree,true);
                    return path;
                }
            }
            path = createPathToGoal(Tree,false);
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

    nav_msgs::Path path = planner.createPath();

    while(ros::ok()) {

        pub.publish(path);

        ros::spinOnce();
        r.sleep();
    }

    
    return 0;

}