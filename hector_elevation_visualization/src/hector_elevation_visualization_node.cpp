#include <ros/ros.h>
#include <hector_elevation_visualization/hector_elevation_visualization.h>

int main (int argc, char** argv)
{
    ros::init(argc,argv,"elevation_visualization");
    ros::NodeHandle node;

    ElevationVisualization elevationVisualization(node);

    return (-1);
} 
