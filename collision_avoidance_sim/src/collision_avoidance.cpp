
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <math.h>

class CollisionAvoidance {
    protected:
        // Sensor subscriber to read the PointClouds
        ros::Subscriber scanSub;
        // Velocity subscriber to control to process the velocity during the
        // the collision avoidance phase
        ros::Subscriber velSub;
        // Joy subscriber to retrieve Joy commands and convert them to Twist
        ros::Subscriber joySub;
        // Velocity publisher
        ros::Publisher velPub;


        ros::NodeHandle nh;

        // Minimum avoidance collision distance
        double radius;
        int count;
        int count_joy;
        int count_pc;


        pcl::PointCloud<pcl::PointXYZ> lastpc;
        
        /*
         * Processing initial velocity command to avoid collision
         * then publishing the processed command to left and right wheels
         *
         * @param msg Initial velocity command
         */
        void velocity_filter(const geometry_msgs::Twist& msg) {
            // Processing Velocity w.r.t the point cloud received
            geometry_msgs::Twist filtered = findClosestAcceptableVelocity(msg);
            velPub.publish(filtered);

        }

        /*
         * Converting data from sensors to messages
         *
         * @param msg PointCloud to convert
         */
        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::fromROSMsg(*msg, lastpc);
            unsigned int n = lastpc.size();
            count_pc += 1;
            //printf("PC number %.3d \n",count_pc);
            // ROS_INFO("New point cloud: %d points",n);
            // for (unsigned int i=0;i<n;i++) {
              //   float x = lastpc[i].x;
                // float y = lastpc[i].y;
                 //float z = lastpc[i].z;
                 //printf("%d %.3f %.3f %.3f",i,x,y,z);
                 //printf("\n");
              //}
            
        }
        
        /*
         * Converting joystick commands to Twist
         *
         * @param Joy command broadcast by the Joystick
         */
        void joy_callback(const sensor_msgs::Joy::ConstPtr& joy) {
		  geometry_msgs::Twist twist;
		  twist.angular.z = 0.7*joy->axes[0];
		  twist.linear.x = 0.3*joy->axes[1];
		  count_joy += 1;
		  velocity_filter(twist);
		  
		}
        
        /*
         * Process velocity  to avoid collision
         *
         * @param desired Initial velocity to filter
         */
        geometry_msgs::Twist findClosestAcceptableVelocity(const geometry_msgs::Twist& desired) {
            geometry_msgs::Twist res = desired;
             unsigned int n = lastpc.size();
			// if robot is close, set linear velocity to 0 (and probably angular too)
			count += 1;
			
			float min_distance2 = 0.2;
			float max_distance2 = 0.6;
	        float pc_x;
	        float pc_y;
	        float current_distance2;
	        float temp=10000;
	        
			for(unsigned int i = 0; i < n; i++) {
				pc_x=lastpc[i].x;
				pc_y=lastpc[i].y;

				if(pc_y < 0.2 && pc_y > -0.2 && pc_x>0){
				// get distance from robot to each point in the point cloud
					current_distance2 = pc_x*pc_x + pc_y*pc_y;
					if (current_distance2 < temp){
						temp = current_distance2;
					}
				}
			}

			if (temp < max_distance2 && res.linear.x > 0){
				// slow down linearly
				res.linear.x *= (temp - min_distance2)/(max_distance2 - min_distance2);
				}
			else if(temp < min_distance2 && res.linear.x > 0){
				// should be stopped if less than minimum distance
				res.linear.x = 0;
				}
				
            return res;
		}
			
			

    public:
        CollisionAvoidance() : nh("~"), radius(1.0) {
            scanSub = nh.subscribe("scans",1,&CollisionAvoidance::pc_callback,this);
            joySub = nh.subscribe("/joy",1, &CollisionAvoidance::joy_callback, this);
            velPub = nh.advertise<geometry_msgs::Twist>("output_vel",1);
            nh.param("radius",radius,1.0);
            nh.param("count",count,0);
            nh.param("count_joy",count_joy,0);
            nh.param("count_pc",count_pc,0);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"collision_avoidance");

    CollisionAvoidance ca;

    ros::spin();
}


