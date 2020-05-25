#include <path_planning/obstacles.h>
#include <geometry_msgs/Point.h>

vector< vector<geometry_msgs::Point> > obstacles::getObstacleArray()
{
    vector<geometry_msgs::Point> obstaclePoint;
    geometry_msgs::Point point;
    
    point.x = 50;
    point.y = 50;
    point.z = 50;

    obstaclePoint.push_back(point);

    
    point.x = 50;
    point.y = 70;
    point.z = 50;

    obstaclePoint.push_back(point);

    
    point.x = 70;
    point.y = 70;
    point.z = 50;

    obstaclePoint.push_back(point);

    
    point.x = 70;
    point.y = 50;
    point.z = 50;
    obstaclePoint.push_back(point);

    
    point.x = 50;
    point.y = 50;
    point.z = 70;
    obstaclePoint.push_back(point);

    
    point.x = 50;
    point.y = 70;
    point.z = 70;
    obstaclePoint.push_back(point);

    
    point.x = 70;
    point.y = 70;
    point.z = 70;
    obstaclePoint.push_back(point);

    
    point.x = 70;
    point.y = 50;
    point.z = 70;
    obstaclePoint.push_back(point);

    
    point.x = 50;
    point.y = 50;
    point.z = 50;
    obstaclePoint.push_back(point);

    obstacleArray.push_back(obstaclePoint);

    return obstacleArray;

}
