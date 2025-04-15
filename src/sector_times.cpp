#include "ros/ros.h"
#include "custom_messages/custom.h"


class sector_times{

public:
    sector_times(){
        sec_status_sub = n.subscribe("/speedsteer", 1, &sector_times::callback, this);
        sec_gps_sub = n.subscribe("/swiftnav/front/gps_pose", 1, &sector_times::callback, this);
        sec_pub = n.advertise<first_project::sector_times>("/sector_times", 1);

        private_n.param("sector1_lat", sec1_lat , 45.630106);
        private_n.param("sector1_longit", sec1_longit, 9.289490);

        private_n.param("sector2_lat", sec2_lat , 45.623570);
        private_n.param("sector2_longit", sec2_lat, 9.287297);

        private_n.param("sector3_lat", sec3_lat , 45.616042);
        private_n.param("sector3_longit", sec3_lat, 9.280767);

        





    }




private:
    
ros::NodeHandle n;
ros::Subscriber sec_status_sub;
ros::Subscriber sec_gps_sub;

ros::Publisher sector_pub = n.advertise<custom_messages::custom>("/sector_times", 1000);

double sector1_time;
double sector2_time;
double sector3_time;
double v_average;

double count = 0;

sector1_time = 0;
sector2_time = 0;
sector3_time = 0;

void callback(const geometry_msgs::PointStamped::ConstPtr& msg1 , const sensor_msgs::NavSatFix::ConstPtr& msg2) {

    count += 1;

    
    double lat = msg2->latitude;
    double longit = msg2->longitude;
    double alt = msg2-> altitude;

    double v = msg1->point.y ; 

    


    custom_messages::custom_msg msg;

    msg.current_sector = 1;

     ros::Time current_time = ros::Time::now();


    if (lat == sec1_lat and longit = sector1_longit){

        msg.current_sector = 2;
        sector1_time = current_time;
        count=1;
        v_average = 0;
        


    }

    if (lat == sec2_lat and longit = sector2_longit){

        msg.current_sector = 3;
        
        sector2_time = current_time;
        count=1;
        v_average = 0;


    }


    if (lat == sec3_lat and longit = sector3_longit){

        msg.current_sector = 1;
        sector3_time = current_time ;
        count=1;
        v_average = 0;



    }

    if (msg.current_sector==2){
        msg.current_sector_time = current_time - sector1_time;
        


    }

    if (msg.current_sector==3){
        msg.current_sector_time = current_time - sector2_time;
        
        

    }

    if (msg.current_sector==1){
        msg.current_sector_time = current_time - sector3_time;
        
        

    }

    v_average = (v+v_average)/count;

    msg.current_sector_mean_speed = v_average;

    sec_pub_.publish(msg);

    



     

    

    

    

    }

};




int main(int argc, char** argv) {
    ros::init(argc, argv, "sector_times");
    sector_times node;
    ros::spin();
    return 0;
}