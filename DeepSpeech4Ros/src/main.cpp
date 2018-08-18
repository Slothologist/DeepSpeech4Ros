
// ros includes
#include "ros/ros.h"

// inludes from this project
#include "../include/utils.h"


utils::config* cfg;

int main(int argc, char *argv[]) {
    // parse config
    cfg = new utils::config();
    read_config(cfg, argv[1]);

    // ros stuff
    ros::init(argc, argv, cfg->ros_node_name);
    ros::NodeHandle n;

    // Jack stuff
}