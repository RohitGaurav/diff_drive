#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Quaternion.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_datatypes.h>
#include<tf/tf.h>
using namespace std;
float t1_,t2_;
class pub_sub
{
	ros::NodeHandle n_;
	public:
		float x_,y_,z_, rx_,ry_,rz_,dx_=0,dy_=0,theta_=0;
		ros::Publisher pub;
		ros::Subscriber sub;
		nav_msgs::Odometry odommsg;
		tf::TransformBroadcaster odom_base;
		pub_sub()
		{			
			sub=n_.subscribe("/cmd_vel", 1000, &pub_sub::cmd_callback,this);
 			
		}
		void cmd_callback(const geometry_msgs::Twist::ConstPtr& velmsg)
		{
			x_=velmsg->linear.x;
			y_=velmsg->linear.y;
			z_=velmsg->linear.z;
			rx_=velmsg->angular.x;			
			ry_=velmsg->angular.y;	
			rz_=velmsg->angular.z;
			theta_=theta_+rz_/10;		
				
			dx_=dx_+(x_*cos(theta_)+y_*sin(theta_))/10;			
			dy_=dy_+(x_*sin(theta_)+y_*cos(theta_))/10;
			theta_=theta_+rz_/10;
			

		}
		void odom_pub(float trans[], float angular)
		{
			pub=n_.advertise<nav_msgs::Odometry>("/odom", 1000);
			geometry_msgs::Quaternion odom_quat;
			odom_quat=tf::createQuaternionMsgFromYaw(angular);
	    		geometry_msgs::TransformStamped odom_trans;
   			odom_trans.header.stamp =ros::Time::now();
    			odom_trans.header.frame_id = "odom";
    			odom_trans.child_frame_id = "base_link";

    			odom_trans.transform.translation.x = trans[0];
    			odom_trans.transform.translation.y =trans[1];
    			odom_trans.transform.translation.z = 0.0;
    			odom_trans.transform.rotation=odom_quat;
			odommsg.header.stamp=ros::Time::now();
			odommsg.header.frame_id="odom";
			
			odommsg.pose.pose.position.x=trans[0];
			odommsg.pose.pose.position.y=trans[1];
			odommsg.pose.pose.orientation = odom_quat;
			odommsg.child_frame_id = "base_link";
   	 		odommsg.twist.twist.linear.x = x_;
    			odommsg.twist.twist.linear.y = 0;
    			odommsg.twist.twist.angular.z = rz_;

			pub.publish(odommsg);
			
		}
		


};


int main( int argc, char **argv)
{


	ros::init(argc,argv,"odom");
	pub_sub ps;
	ros::Rate rate(10);
	float trans[2];
	float angle;
	
	while(ros::ok())
	{	//cout<<ps.x_<<'|'<<ps.y_<<'|'<<ps.z_<<endl; 
		//cout<<ps.rx_<<'|'<<ps.ry_<<'|'<<ps.rz_<<endl;
		trans[0]=ps.dx_;
		trans[1]=ps.dy_;
		angle=ps.theta_;
		ps.odom_pub(trans,angle);
		ros::spinOnce();
		rate.sleep();
			
	}

return 0;
}
