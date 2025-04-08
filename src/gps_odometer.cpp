#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

class OdometryNode{
    private:
    
    ros::NodeHandle nh_;
    ros::Subscriber gp_odom_sub_;
    ros::Publisher gps_odom_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    

};




int main(int argc, char** argv) {
    ros::init(argc, argv, "bicycle_odometry_node");
    OdometryNode node;
    ros::spin();
    return 0;
}