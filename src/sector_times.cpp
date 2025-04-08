#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

class boh{
    private:
    
    ros::NodeHandle nh_;
    ros::Subscriber gp_odom_sub_;
    ros::Publisher gps_odom_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    

};




int main(int argc, char** argv) {
    ros::init(argc, argv, "bicycle_odometry_node");
    boh node;
    ros::spin();
    return 0;
}