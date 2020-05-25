#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <path_planning/rrt.h>
#include <path_planning/obstacles.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>


using namespace rrt;

void initializeMarkers(visualization_msgs::Marker &boundary,
    visualization_msgs::Marker &obstacle,
    visualization_msgs::Marker &boundary2,
    visualization_msgs::Marker &boundary3,
    visualization_msgs::Marker &boundary4,
    visualization_msgs::Marker &boundary5,
    visualization_msgs::Marker &boundary6,
    visualization_msgs::Marker &obstacle2,
    visualization_msgs::Marker &obstacle3,
    visualization_msgs::Marker &obstacle4,
    visualization_msgs::Marker &obstacle5,
    visualization_msgs::Marker &obstacle6)
{
	boundary.header.frame_id    = obstacle.header.frame_id    = obstacle2.header.frame_id    = obstacle3.header.frame_id    = obstacle4.header.frame_id    = obstacle5.header.frame_id    = obstacle6.header.frame_id    =  "path_planner";
	boundary.header.stamp       = obstacle.header.stamp       = obstacle2.header.stamp       = obstacle3.header.stamp       = obstacle4.header.stamp       = obstacle5.header.stamp       = obstacle6.header.stamp       =  ros::Time::now();
	boundary.ns                 = obstacle.ns                 = obstacle2.ns                 = obstacle3.ns                 = obstacle4.ns                 = obstacle5.ns                 = obstacle6.ns                 =  "path_planner";
	boundary.action             = obstacle.action             = obstacle2.action             = obstacle3.action             = obstacle4.action             = obstacle5.action             = obstacle6.action             =  visualization_msgs::Marker::ADD;
	boundary.pose.orientation.w = obstacle.pose.orientation.w = obstacle2.pose.orientation.w = obstacle3.pose.orientation.w = obstacle4.pose.orientation.w = obstacle5.pose.orientation.w = obstacle6.pose.orientation.w =  1.0;

    boundary.id    = 110;
	obstacle.id   = 111;
    obstacle2.id   = 117;
    obstacle3.id   = 118;
    obstacle4.id   = 119;
    obstacle5.id   = 120;
    obstacle6.id   = 121;

	boundary.scale.x = 1;
	boundary.type  = visualization_msgs::Marker::LINE_STRIP;
	obstacle.type = visualization_msgs::Marker::CUBE;
    obstacle2.type = visualization_msgs::Marker::CUBE;
    obstacle3.type = visualization_msgs::Marker::CUBE;
    obstacle4.type = visualization_msgs::Marker::CUBE;
    obstacle5.type = visualization_msgs::Marker::CUBE;
    obstacle6.type = visualization_msgs::Marker::CUBE;
  
    obstacle.pose.position.x = 20;
    obstacle.pose.position.y = 20;
    obstacle.pose.position.z = 50;
    
    obstacle.scale.x = 10;
    obstacle.scale.y = 10;
    obstacle.scale.z = 100;
    
    obstacle.color.r = 0.64f;
    obstacle.color.g = 0.57f;
    obstacle.color.b = 0.38f;
 
    obstacle2.pose.position.x = 30;
    obstacle2.pose.position.y = 40;
    obstacle2.pose.position.z = 30;
    
    obstacle2.scale.x = 14;
    obstacle2.scale.y = 14;
    obstacle2.scale.z = 60;
    
    obstacle2.color.r = 0.80f;
    obstacle2.color.g = 0.66f;
    obstacle2.color.b = 0.78f;

    obstacle3.pose.position.x = 40;
    obstacle3.pose.position.y = 85;
    obstacle3.pose.position.z = 35;
    
    obstacle3.scale.x = 20;
    obstacle3.scale.y = 10;
    obstacle3.scale.z = 70;
    
    obstacle3.color.r = 0.14f;
    obstacle3.color.g = 0.67f;
    obstacle3.color.b = 0.98f;

    obstacle4.pose.position.x = 65;
    obstacle4.pose.position.y = 45;
    obstacle4.pose.position.z = 50;
    
    obstacle4.scale.x = 10;
    obstacle4.scale.y = 30;
    obstacle4.scale.z = 100;
    
    obstacle4.color.r = 0.14f;
    obstacle4.color.g = 0.17f;
    obstacle4.color.b = 0.38f;

    obstacle5.pose.position.x = 65;
    obstacle5.pose.position.y = 85;
    obstacle5.pose.position.z = 45;
    
    obstacle5.scale.x = 10;
    obstacle5.scale.y = 10;
    obstacle5.scale.z = 90;
    
    obstacle5.color.r = 0.94f;
    obstacle5.color.g = 0.57f;
    obstacle5.color.b = 0.98f;

    obstacle6.pose.position.x = 85;
    obstacle6.pose.position.y = 75;
    obstacle6.pose.position.z = 50;
    
    obstacle6.scale.x = 10;
    obstacle6.scale.y = 10;
    obstacle6.scale.z = 100;
    
    obstacle6.color.r = 0.64f;
    obstacle6.color.g = 0.97f;
    obstacle6.color.b = 0.68f;
    
	boundary.color.r = 0.0f;
	boundary.color.g = 0.0f;
	boundary.color.b = 0.0f;

	boundary.color.a = obstacle.color.a = obstacle2.color.a = obstacle3.color.a = obstacle4.color.a = obstacle5.color.a = obstacle6.color.a = 1.0f;
    boundary2.header.frame_id = boundary3.header.frame_id = boundary4.header.frame_id = boundary5.header.frame_id = boundary6.header.frame_id = "path_planner";
    boundary2.header.stamp = boundary3.header.stamp = boundary4.header.stamp = boundary5.header.stamp = boundary6.header.stamp = ros::Time::now();
    boundary2.ns = boundary3.ns = boundary4.ns = boundary5.ns = boundary6.ns = "path_planner";
    boundary2.action = boundary3.action = boundary4.action = boundary5.action = boundary6.action= visualization_msgs::Marker::ADD;
    boundary2.pose.orientation.w = boundary3.pose.orientation.w = boundary4.pose.orientation.w = boundary5.pose.orientation.w = boundary6.pose.orientation.w = 1.0;

    boundary2.id = 112;
    boundary3.id = 113;
    boundary4.id = 114;
    boundary5.id = 115;
    boundary6.id = 116;

    boundary2.type = boundary3.type = boundary4.type = boundary5.type = boundary6.type = visualization_msgs::Marker::LINE_STRIP;

    boundary2.scale.x = boundary3.scale.x = boundary4.scale.x = boundary5.scale.x = boundary6.scale.x = 1;

    boundary2.color.r = boundary3.color.r = boundary4.color.r = boundary5.color.r = boundary6.color.r = 0.0f;
	boundary2.color.g = boundary3.color.g = boundary4.color.g = boundary5.color.g = boundary6.color.g = 0.0f;
	boundary2.color.b = boundary3.color.b = boundary4.color.b = boundary5.color.b = boundary6.color.b = 0.0f;
    boundary2.color.a = boundary3.color.a = boundary4.color.a = boundary5.color.a = boundary6.color.a = 1.0f;
    
}

vector<geometry_msgs::Point> initializeBoundary()
{
    vector<geometry_msgs::Point> bondArray;

    geometry_msgs::Point point;

    
    point.x = 0;
    point.y = 0;
    point.z = 0;

    bondArray.push_back(point);

    
    point.x = 0;
    point.y = 100;
    point.z = 0;

    bondArray.push_back(point);

    
    point.x = 100;
    point.y = 100;
    point.z = 0;

    bondArray.push_back(point);

    
    point.x = 100;
    point.y = 0;
    point.z = 0;
    bondArray.push_back(point);

    
    point.x = 0;
    point.y = 0;
    point.z = 0;
    bondArray.push_back(point);

    return bondArray;
}


vector<geometry_msgs::Point> initializeBoundary2()
{
    vector<geometry_msgs::Point> bondArray;

    geometry_msgs::Point point;

    
    point.x = 0;
    point.y = 0;
    point.z = 0;

    bondArray.push_back(point);

    
    point.x = 0;
    point.y = 0;
    point.z = 100;

    bondArray.push_back(point);

    
    point.x = 0;
    point.y = 100;
    point.z = 100;

    bondArray.push_back(point);

    
    point.x = 0;
    point.y = 100;
    point.z = 0;
    bondArray.push_back(point);

    
    point.x = 0;
    point.y = 0;
    point.z = 0;
    bondArray.push_back(point);

    return bondArray;
}
vector<geometry_msgs::Point> initializeBoundary3()
{
    vector<geometry_msgs::Point> bondArray;

    geometry_msgs::Point point;

    
    point.x = 0;
    point.y = 0;
    point.z = 0;

    bondArray.push_back(point);

    
    point.x = 0;
    point.y = 0;
    point.z = 100;

    bondArray.push_back(point);

    
    point.x = 100;
    point.y = 0;
    point.z = 100;

    bondArray.push_back(point);

    
    point.x = 100;
    point.y = 0;
    point.z = 0;
    bondArray.push_back(point);

    
    point.x = 0;
    point.y = 0;
    point.z = 0;
    bondArray.push_back(point);

    return bondArray;
}
vector<geometry_msgs::Point> initializeBoundary4()
{
    vector<geometry_msgs::Point> bondArray;

    geometry_msgs::Point point;

    
    point.x = 0;
    point.y = 0;
    point.z = 100;

    bondArray.push_back(point);

    
    point.x = 0;
    point.y = 100;
    point.z = 100;

    bondArray.push_back(point);

    
    point.x = 100;
    point.y = 100;
    point.z = 100;

    bondArray.push_back(point);

    
    point.x = 100;
    point.y = 0;
    point.z = 100;
    bondArray.push_back(point);

    
    point.x = 0;
    point.y = 0;
    point.z = 100;
    bondArray.push_back(point);

    return bondArray;
}
vector<geometry_msgs::Point> initializeBoundary5()
{
    vector<geometry_msgs::Point> bondArray;

    geometry_msgs::Point point;

    
    point.x = 100;
    point.y = 0;
    point.z = 0;

    bondArray.push_back(point);

    
    point.x = 100;
    point.y = 0;
    point.z = 100;

    bondArray.push_back(point);

    
    point.x = 100;
    point.y = 100;
    point.z = 100;

    bondArray.push_back(point);

    
    point.x = 100;
    point.y = 100;
    point.z = 0;
    bondArray.push_back(point);

    
    point.x = 100;
    point.y = 0;
    point.z = 0;
    bondArray.push_back(point);

    return bondArray;
}
vector<geometry_msgs::Point> initializeBoundary6()
{
    vector<geometry_msgs::Point> bondArray;

    geometry_msgs::Point point;

    
    point.x = 0;
    point.y = 100;
    point.z = 0;

    bondArray.push_back(point);

    
    point.x = 0;
    point.y = 100;
    point.z = 100;

    bondArray.push_back(point);

    
    point.x = 100;
    point.y = 100;
    point.z = 100;

    bondArray.push_back(point);

    
    point.x = 100;
    point.y = 100;
    point.z = 0;
    bondArray.push_back(point);

    
    point.x = 0;
    point.y = 100;
    point.z = 0;
    bondArray.push_back(point);

    return bondArray;
}

vector<geometry_msgs::Point> initializeObstacles()
{
    vector< vector<geometry_msgs::Point> > obstArray;
    vector<geometry_msgs::Point> obstaclesMarker;
    obstacles obst;
    obstArray = obst.getObstacleArray();

    for(int i=0; i<obstArray.size(); i++)
    {
        for(int j=1; j<9; j++)
        {
            obstaclesMarker.push_back(obstArray[i][j-1]);
            obstaclesMarker.push_back(obstArray[i][j]);
            
        }      
    }
    return obstaclesMarker;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"env_node");
	ros::NodeHandle n;

	ros::Publisher env_publisher = n.advertise<visualization_msgs::Marker>("path_planner_rrt",1);

    visualization_msgs::Marker boundary;
    visualization_msgs::Marker obstacle;
    visualization_msgs::Marker obstacle2;
    visualization_msgs::Marker obstacle3;
    visualization_msgs::Marker obstacle4;
    visualization_msgs::Marker obstacle5;
    visualization_msgs::Marker obstacle6;
    
    visualization_msgs::Marker boundary2;
    visualization_msgs::Marker boundary3;
    visualization_msgs::Marker boundary4;
    visualization_msgs::Marker boundary5;
    visualization_msgs::Marker boundary6;

    initializeMarkers(boundary, obstacle, boundary2, boundary3, boundary4, boundary5, boundary6, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6);

    RRT myRRT(2.0, 2.0, 2.0);

    boundary.points = initializeBoundary();
    boundary2.points = initializeBoundary2();
    boundary3.points = initializeBoundary3();
    boundary4.points = initializeBoundary4();
    boundary5.points = initializeBoundary5();
    boundary6.points = initializeBoundary6();

    env_publisher.publish(boundary);
    env_publisher.publish(obstacle);
    env_publisher.publish(obstacle2);
    env_publisher.publish(obstacle3);
    env_publisher.publish(obstacle4);
    env_publisher.publish(obstacle5);
    env_publisher.publish(obstacle6);
    env_publisher.publish(boundary2);
    env_publisher.publish(boundary3);
    env_publisher.publish(boundary4);
    env_publisher.publish(boundary5);
    env_publisher.publish(boundary6);
    while(ros::ok())
    {
        env_publisher.publish(boundary);
        env_publisher.publish(obstacle);
        env_publisher.publish(obstacle2);
        env_publisher.publish(obstacle3);
        env_publisher.publish(obstacle4);
        env_publisher.publish(obstacle5);
        env_publisher.publish(obstacle6);
        env_publisher.publish(boundary2);
        env_publisher.publish(boundary3);
        env_publisher.publish(boundary4);
        env_publisher.publish(boundary5);
        env_publisher.publish(boundary6);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
	return 1;
}
