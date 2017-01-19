#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <sound_play/sound_play.h>
#include <fstream>
using namespace cv;
int point_ite=0;
ros::Publisher vis_pub;
geometry_msgs::PointStamped pnt[4];
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void _marker_publish(double& x, double& y, int& mark)
{
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.ns = "simple_navigation_goals";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.001;
    if(mark == 6)
    {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
    }
    else
    {
        marker.color.r = 0.0;
        marker.color.g = 0.5;
    }
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 0;
    marker.lifetime = ros::Duration();
    marker.header.frame_id = "map";
    marker.id = mark;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    vis_pub.publish(marker);
    ros::Duration(0.01).sleep();
}
void _col_det () 
{
    sound_play::SoundClient sc;
    float pix_max=0;
    VideoCapture cap;
    if(!cap.open(1))
    {
        ROS_INFO_ONCE("Could not find camera");
        return;
    }
    Mat frame;
    cap >> frame;
    if(frame.empty()){return;}
    for(int pix_rows=0; pix_rows<frame.rows; pix_rows++)
    {
        for(int pix_cols=0; pix_cols<frame.cols; pix_cols++)
        {
            Vec3b intensity = frame.at<Vec3b>(pix_rows, pix_cols);
            int blue = intensity.val[0];
            int green = intensity.val[1];
            int red = intensity.val[2];
            if (red > 85 && green < 40 && blue < 55){pix_max++;}
        }
    }
	float percent = pix_max/(frame.rows*frame.cols)*100;
	if (percent>3)
	{
		sc.playWave("/home/b219/catkin_axel/src/simple_navigation_goals/turret_sounds/5.wav");
		imwrite("test.jpg", frame);
		tf::TransformListener listener;
		double origin_x, origin_y;
		while (percent>3)
		{
			tf::StampedTransform transform;
			try
			{
				listener.lookupTransform("/map","/base_link",ros::Time(0), transform);
				origin_x = transform.getOrigin().x(), origin_y = transform.getOrigin().y();
				break;
			}
			catch (tf::TransformException ex){ROS_ERROR("Nope! %s", ex.what());}
		}
		int mark_temp = 6;
		_marker_publish (origin_x, origin_y, mark_temp);
	}
}
void _clicked_point(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    pnt[point_ite]= *msg;
    point_ite++;
    if(point_ite>3)
    {
        double point_x[6];
        double point_y[6];
        for(int div=0;div<=2;div++)
        {
            int i[6]={0,3,4,1,2,5};
            point_x[i[div]]=pnt[2].point.x+(pnt[0].point.x-pnt[2].point.x)*(div*2+1)/6;
            point_y[i[div]]=pnt[2].point.y+(pnt[0].point.y-pnt[2].point.y)*(div*2+1)/6;
            point_x[i[div+3]]=pnt[3].point.x+(pnt[1].point.x-pnt[3].point.x)*(div*2+1)/6;
            point_y[i[div+3]]=pnt[3].point.y+(pnt[1].point.y-pnt[3].point.y)*(div*2+1)/6;
        }
        for(int mark=0; mark<6; mark++){_marker_publish (point_x[mark], point_y[mark], mark);}    
        for(int pub_pnt=0;pub_pnt<6;pub_pnt++)
        {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){ROS_INFO("Missing ROS services");}         
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = point_x[pub_pnt];
            goal.target_pose.pose.position.y = point_y[pub_pnt];
            goal.target_pose.pose.orientation.w=1.0;
            ROS_INFO("Sending goal");
            ac.sendGoal(goal);
            while(ac.getState() == actionlib::SimpleClientGoalState::PENDING)
            {
                while(ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
                {
                    _col_det ();
                }
            }
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){ROS_INFO("Success");}
            else
	        {
	            ROS_INFO("Retrying");
	            ros::Duration(0.5).sleep();
	            pub_pnt--;		
	        }
        }
    }
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle n;
    vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
    ros::Subscriber sub = n.subscribe("clicked_point", 1000, _clicked_point);
    ros::spin();
}