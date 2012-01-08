#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rostweet_msgs/postTweet.h>

int main( int argc, char* argv[] )
{
   if (argc<2 || argc>3) {
   	std::cerr << "Usage: " << argv[0] << " \"Text to tweet\" [<image_file>]" << std::endl;
	exit(0);
   }
   std::string tweetText(argv[1]);

   ros::init(argc, argv, "post");
   ros::NodeHandle nh("~");
   
   ros::ServiceClient client = nh.serviceClient<rostweet_msgs::postTweet>("/rostweet/postTweet");

  rostweet_msgs::postTweet srv;
  srv.request.text = tweetText;

   if (argc>2) {
	std::string tweetPic(argv[2]);
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
	cv_ptr->image = cv::imread(tweetPic);
	cv_ptr->encoding=std::string("bgr8");
	if (cv_ptr->image.data!=NULL) {
  		srv.request.picture.push_back(*(cv_ptr->toImageMsg()));
	} else {
		std::cerr << "Error: cannot load image " << tweetPic << " from disk" << std::endl;
		return 0;
	}
   }
   ROS_INFO("Calling service...");
   if (!client.call(srv))
    ROS_ERROR("Failed to call service.");
   else {
	if (srv.response.result) {
		ROS_INFO("Posted successfully");
	} else {
		ROS_ERROR("Error posting. See rostweet error messages");
	}
   }

   return 0;
}
