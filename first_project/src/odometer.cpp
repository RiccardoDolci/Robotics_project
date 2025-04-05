#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

class OdometryNode {
public:
    OdometryNode() {
        // Subscribe al topic /speedsteer per ottenere i dati dal veicolo
        sub_ = nh_.subscribe("/speedsteer", 1, &Odometer::callback, this);
        
        // Publisher per l'odometria
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
    }

    void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        // Costruzione del messaggio di odometria
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "vehicle";

        
        double steering = msg->point.x;
        double velocity = msg->point.y;

        // Esegui l'integrazione per aggiornare la posizione (placeholder)
        // In un caso reale dovresti usare un modello cinetico per calcolare la posizione e l'orientamento.
        odom_msg.pose.pose.position.x = 0.0;  // Placeholder
        odom_msg.pose.pose.position.y = 0.0;  // Placeholder
        odom_msg.pose.pose.position.z = 0.0;  // Placeholder

        // Popola la velocità
        odom_msg.twist.twist.linear.x = velocity;
        odom_msg.twist.twist.angular.z = steering;

        // Pubblica il messaggio di odometria
        odom_pub_.publish(odom_msg);

        // Crea il tf per il legame tra "odom" e "vehicle"
        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = ros::Time::now();
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "vehicle";

        // Placeholder per la posizione del veicolo (in un caso reale sarebbe aggiornata)
        odom_tf.transform.translation.x = 0.0;
        odom_tf.transform.translation.y = 0.0;
        odom_tf.transform.translation.z = 0.0;

        // Imposta la rotazione (orientamento) del veicolo come 0 (placeholder)
        tf::Quaternion q;
        q.setRPY(0, 0, 0);  // Nessuna rotazione, placeholder
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();

        // Broadcast del tf
        tf_broadcaster_.sendTransform(odom_tf);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometer");
    OdometryNode node;  // Crea l'istanza del nodo
    ros::spin();    // Resta in ascolto dei messaggi
    return 0;
}



