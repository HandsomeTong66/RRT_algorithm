#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <path_planning/rrt.h>
#include <path_planning/obstacles.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <time.h>

#define success false
#define running true

using namespace rrt;

bool status = running;

void initializeMarkers(visualization_msgs::Marker &sourcePoint,
    visualization_msgs::Marker &goalPoint,
    visualization_msgs::Marker &randomPoint,
    visualization_msgs::Marker &rrtTreeMarker,
    visualization_msgs::Marker &finalPath)
{
    
	sourcePoint.header.frame_id    = goalPoint.header.frame_id    = randomPoint.header.frame_id    = rrtTreeMarker.header.frame_id    = finalPath.header.frame_id    = "path_planner";
	sourcePoint.header.stamp       = goalPoint.header.stamp       = randomPoint.header.stamp       = rrtTreeMarker.header.stamp       = finalPath.header.stamp       = ros::Time::now();
	sourcePoint.ns                 = goalPoint.ns                 = randomPoint.ns                 = rrtTreeMarker.ns                 = finalPath.ns                 = "path_planner";
	sourcePoint.action             = goalPoint.action             = randomPoint.action             = rrtTreeMarker.action             = finalPath.action             = visualization_msgs::Marker::ADD;
	sourcePoint.pose.orientation.w = goalPoint.pose.orientation.w = randomPoint.pose.orientation.w = rrtTreeMarker.pose.orientation.w = finalPath.pose.orientation.w = 1.0;

    
    sourcePoint.id    = 0;
	goalPoint.id      = 1;
	randomPoint.id    = 2;
	rrtTreeMarker.id  = 3;
    finalPath.id      = 4;

	
	rrtTreeMarker.type                                    = visualization_msgs::Marker::LINE_LIST;
	finalPath.type                                        = visualization_msgs::Marker::LINE_STRIP;
	sourcePoint.type  = goalPoint.type = randomPoint.type = visualization_msgs::Marker::SPHERE;

	
	rrtTreeMarker.scale.x = 0.2;
	finalPath.scale.x     = 1;

    sourcePoint.scale.x   = goalPoint.scale.x = 2;
    sourcePoint.scale.y   = goalPoint.scale.y = 2;
    sourcePoint.scale.z   = goalPoint.scale.z = 1;

    randomPoint.scale.x = 2;
    randomPoint.scale.y = 2;
    randomPoint.scale.z = 1;

	sourcePoint.color.r   = 1.0f;
	goalPoint.color.g     = 1.0f;

    randomPoint.color.b   = 1.0f;

	rrtTreeMarker.color.r = 0.8f;
	rrtTreeMarker.color.g = 0.4f;

	finalPath.color.r = 0.2f;
	finalPath.color.g = 0.2f;
	finalPath.color.b = 1.0f;

	sourcePoint.color.a = goalPoint.color.a = randomPoint.color.a = rrtTreeMarker.color.a = finalPath.color.a = 1.0f;
}

vector< vector<geometry_msgs::Point> > getObstacles()
{
    obstacles obst;
    return obst.getObstacleArray();
}


void addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, RRT::rrtNode &tempNode, RRT &myRRT)
{

    geometry_msgs::Point point;

    point.x = tempNode.posX;
    point.y = tempNode.posY;
    point.z = tempNode.posZ;
    rrtTreeMarker.points.push_back(point);


    RRT::rrtNode parentNode = myRRT.getParent(tempNode.nodeID);

    point.x = parentNode.posX;
    point.y = parentNode.posY;
    point.z = parentNode.posZ;

    rrtTreeMarker.points.push_back(point);


}

bool checkIfInsideBoundary(RRT::rrtNode &tempNode)
{
    if(tempNode.posX < 0 || tempNode.posY < 0  || tempNode.posZ < 0 || tempNode.posX > 100 || tempNode.posY > 100 || tempNode.posZ > 100) return false;
    else return true;
}

bool checkIfOutsideObstacles(vector< vector<geometry_msgs::Point> > &obstArray, RRT::rrtNode &tempNode)
{
    double AB, AD, AMAB, AMAD;

    for(int i=0; i<obstArray.size(); i++)
    {
        
        AB = (pow(obstArray[i][0].x - obstArray[i][1].x,2) + pow(obstArray[i][0].y - obstArray[i][1].y,2));
        AD = (pow(obstArray[i][0].x - obstArray[i][3].x,2) + pow(obstArray[i][0].y - obstArray[i][3].y,2));

        
        AMAB = (((tempNode.posX - obstArray[i][0].x) * (obstArray[i][1].x - obstArray[i][0].x)) + (( tempNode.posY - obstArray[i][0].y) * (obstArray[i][1].y - obstArray[i][0].y)));
        AMAD = (((tempNode.posX - obstArray[i][0].x) * (obstArray[i][3].x - obstArray[i][0].x)) + (( tempNode.posY - obstArray[i][0].y) * (obstArray[i][3].y - obstArray[i][0].y)));

        if((0 < AMAB) && (AMAB < AB) && (0 < AMAD) && (AMAD < AD))
        {
            return false;
        }
    }
    return true;
}


bool checkIfOutsideObstacles3D(RRT::rrtNode &nearesetNode, RRT::rrtNode &tempNode)
{
    double length = abs((nearesetNode.posX - tempNode.posX)); 
    double width = abs((nearesetNode.posY - tempNode.posY));
    double heigth = abs((nearesetNode.posZ - tempNode.posZ));
    RRT::rrtNode bodyCenter;
    bodyCenter.posX = (nearesetNode.posX + tempNode.posX)/2;
    bodyCenter.posY = (nearesetNode.posY + tempNode.posY)/2;
    bodyCenter.posZ = (nearesetNode.posZ + tempNode.posZ)/2;

    vector<geometry_msgs::Point> vertices;
    geometry_msgs::Point vertice;
    
    vertice.x = bodyCenter.posX - length/2;
    vertice.y = bodyCenter.posY - width/2;
    vertice.z = bodyCenter.posZ - heigth/2;
    vertices.push_back(vertice);

    
    vertice.x = bodyCenter.posX - length/2;
    vertice.y = bodyCenter.posY + width/2;
    vertice.z = bodyCenter.posZ - heigth/2;
    vertices.push_back(vertice);

    
    vertice.x = bodyCenter.posX + length/2;
    vertice.y = bodyCenter.posY + width/2;
    vertice.z = bodyCenter.posZ - heigth/2;
    vertices.push_back(vertice);

    
    vertice.x = bodyCenter.posX + length/2;
    vertice.y = bodyCenter.posY - width/2;
    vertice.z = bodyCenter.posZ - heigth/2;
    vertices.push_back(vertice);

    
    vertice.x = bodyCenter.posX - length/2;
    vertice.y = bodyCenter.posY - width/2;
    vertice.z = bodyCenter.posZ + heigth/2;
    vertices.push_back(vertice);

    
    vertice.x = bodyCenter.posX - length/2;
    vertice.y = bodyCenter.posY + width/2;
    vertice.z = bodyCenter.posZ + heigth/2;
    vertices.push_back(vertice);
    
    
    vertice.x = bodyCenter.posX + length/2;
    vertice.y = bodyCenter.posY + width/2;
    vertice.z = bodyCenter.posZ + heigth/2;
    vertices.push_back(vertice);

    
    vertice.x = bodyCenter.posX + length/2;
    vertice.y = bodyCenter.posY - width/2;
    vertice.z = bodyCenter.posZ + heigth/2;
    vertices.push_back(vertice);

    for(int i=0; i < vertices.size(); i++)
    {
        if((vertices[i].x > 15) && (vertices[i].x < 25) && (vertices[i].y > 15) && (vertices[i].y < 25) && (vertices[i].z > 0) && (vertices[i].z < 100) ||
           (vertices[i].x > 23) && (vertices[i].x < 37) && (vertices[i].y > 33) && (vertices[i].y < 47) && (vertices[i].z > 0) && (vertices[i].z < 60) ||
           (vertices[i].x > 30) && (vertices[i].x < 50) && (vertices[i].y > 80) && (vertices[i].y < 90) && (vertices[i].z > 0) && (vertices[i].z < 70) ||
           (vertices[i].x > 60) && (vertices[i].x < 70) && (vertices[i].y > 30) && (vertices[i].y < 60) && (vertices[i].z > 0) && (vertices[i].z < 100) ||
           (vertices[i].x > 60) && (vertices[i].x < 70) && (vertices[i].y > 80) && (vertices[i].y < 90) && (vertices[i].z > 0) && (vertices[i].z < 90) ||
           (vertices[i].x > 80) && (vertices[i].x < 90) && (vertices[i].y > 70) && (vertices[i].y < 80) && (vertices[i].z > 0) && (vertices[i].z < 100))
        return false;
    }
    return true;
}

void generateTempPoint(RRT::rrtNode &tempNode)
{
    int x = rand() % 150 + 1;
    int y = rand() % 150 + 1;
    int z = rand() % 150 + 1;
    
    tempNode.posX = x;
    tempNode.posY = y;
    tempNode.posZ = z;
}

bool addNewPointtoRRT(RRT &myRRT, RRT::rrtNode &tempNode, int rrtStepSize, vector< vector<geometry_msgs::Point> > &obstArray)
{
    int nearestNodeID = myRRT.getNearestNodeID(tempNode.posX, tempNode.posY, tempNode.posZ);

    RRT::rrtNode nearestNode = myRRT.getNode(nearestNodeID);

    double theta1 = atan2(tempNode.posZ - nearestNode.posZ, sqrt(pow(tempNode.posX - nearestNode.posX, 2) + pow(tempNode.posY - nearestNode.posY, 2) ));
    double theta2 = atan2(sqrt(pow(tempNode.posY - nearestNode.posY, 2) + pow(tempNode.posZ - nearestNode.posZ, 2)), tempNode.posX - nearestNode.posX);
    double theta3 = atan2(sqrt(pow(tempNode.posZ - nearestNode.posZ, 2) + pow(tempNode.posX - nearestNode.posX, 2)), tempNode.posY - nearestNode.posY);

    tempNode.posZ = nearestNode.posZ + (1 * sin(theta1));
    tempNode.posY = nearestNode.posY + (rrtStepSize * cos(theta3));
    tempNode.posX = nearestNode.posX + (rrtStepSize * cos(theta2));

    if(checkIfInsideBoundary(tempNode) && checkIfOutsideObstacles3D(nearestNode, tempNode))
    {
        
        tempNode.parentID = nearestNodeID;
        tempNode.nodeID = myRRT.getTreeSize();
        myRRT.addNewNode(tempNode);
        return true;
    }
    else
        return false;
}


bool checkNodetoGoal(int X, int Y, int Z, RRT::rrtNode &tempNode)
{
    double distance = sqrt(pow(X-tempNode.posX,2)+pow(Y-tempNode.posY,2)+pow(Z-tempNode.posZ,2));
    if(distance < 10)
    {
        return true;
    }
    return false;
}

void setFinalPathData(vector< vector<int> > &rrtPaths, RRT &myRRT, int i, visualization_msgs::Marker &finalpath, int goalX, int goalY, int goalZ)
{ 
    RRT::rrtNode tempNode;
    geometry_msgs::Point point;
    for(int j=0; j<rrtPaths[i].size();j++)
    {
        tempNode = myRRT.getNode(rrtPaths[i][j]);
        point.x = tempNode.posX;
        point.y = tempNode.posY;
        point.z = tempNode.posZ;

        finalpath.points.push_back(point);
    }

    point.x = goalX;
    point.y = goalY;
    point.z = goalZ;
    finalpath.points.push_back(point);
}
void displayTheFinalPathNodeInfo(vector<int> path, RRT &myRRT)
{
    for(int i=0; i<path.size(); i++)
    {
        RRT::rrtNode testNode = myRRT.getNode(path[i]);
        std::cout << "path[i] = " << path[i] << std::endl;
        std::cout << "testNode.nodeID = " << testNode.nodeID << std::endl;
        std::cout << "testNode.parentID = " << testNode.parentID << std::endl;
        std::cout << "testNode.posX = " << testNode.posX << std::endl;
        std::cout << "testNode.posY = " << testNode.posY << std::endl;
        std::cout << "testNode.posZ = " << testNode.posZ << std::endl;
        std::cout << "------------" << std::endl;
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"rrt_node");
	ros::NodeHandle n;

	ros::Publisher rrt_publisher = n.advertise<visualization_msgs::Marker>("path_planner_rrt",1);

    visualization_msgs::Marker sourcePoint;
    visualization_msgs::Marker goalPoint;
    visualization_msgs::Marker randomPoint;
    visualization_msgs::Marker rrtTreeMarker;
    visualization_msgs::Marker finalPath;

    initializeMarkers(sourcePoint, goalPoint, randomPoint, rrtTreeMarker, finalPath);

    
    sourcePoint.pose.position.x = 2;
    sourcePoint.pose.position.y = 2;
    sourcePoint.pose.position.z = 2;
    
    goalPoint.pose.position.x = 95;
    goalPoint.pose.position.y = 95;
    goalPoint.pose.position.z = 30;

    rrt_publisher.publish(sourcePoint);
    rrt_publisher.publish(goalPoint);
    ros::spinOnce();
    ros::Duration(0.01).sleep();

    srand (time(NULL));

    RRT myRRT(2.0, 2.0, 2.0);
    int goalX, goalY, goalZ;
    goalX = goalY = 95;
    goalZ = 30;

    int rrtStepSize = 3;

    vector< vector<int> > rrtPaths;
    
    vector<int> path;
    int rrtPathLimit = 1;

    int shortestPathLength = 9999;
    int shortestPath = -1;

    RRT::rrtNode tempNode;

    vector< vector<geometry_msgs::Point> >  obstacleList = getObstacles();

    bool addNodeResult = false, nodeToGoal = false;

    while(ros::ok() && status)
    {
        if(rrtPaths.size() < rrtPathLimit)
        {

            generateTempPoint(tempNode);
            

            addNodeResult = addNewPointtoRRT(myRRT,tempNode,rrtStepSize,obstacleList);
            
            if(addNodeResult)
            {               
                addBranchtoRRTTree(rrtTreeMarker,tempNode,myRRT);
               
                nodeToGoal = checkNodetoGoal(goalX, goalY, goalZ, tempNode);
                
                if(nodeToGoal)
                {
                    path = myRRT.getRootToEndPath(tempNode.nodeID);
                    displayTheFinalPathNodeInfo(path, myRRT);
                    rrtPaths.push_back(path);
                    std::cout<<"New Path Found. Total paths "<<rrtPaths.size()<<endl;
                }
            }
        }
        else 
        {
            status = success; 
            std::cout<<"Finding Optimal Path"<<endl;
            for(int i=0; i<rrtPaths.size();i++)
            {         
                if(rrtPaths[i].size() < shortestPath)
                {     
                    shortestPath = i;
                    shortestPathLength = rrtPaths[i].size();
                }
            }  
            setFinalPathData(rrtPaths, myRRT, shortestPath, finalPath, goalX, goalY, goalZ);
            rrt_publisher.publish(finalPath);
        }
        rrt_publisher.publish(sourcePoint);
        rrt_publisher.publish(goalPoint);
        rrt_publisher.publish(rrtTreeMarker);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
	return 1;
}
