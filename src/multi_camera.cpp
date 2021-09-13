/**
 * multi_camera.cpp
 *
 * Copyright (C) 2019 Alexis Guijarro <toton95@hotmail.com>
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version
 * 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General
 * Public License along with this program. If not, see
 * <https://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>


//Variable that contains the name of the window
static const std::string OPENCV_WINDOW = "Multi camera viewer";

class multiVideo
{
	//Nodehandle that is in charge of every operation and the image transport that acquires and publishes the images back to ROS
	ros::NodeHandle n;
	image_transport::ImageTransport it;
	image_transport::Subscriber img_sub_1;
	image_transport::Subscriber img_sub_2;
	image_transport::Publisher img_pub;

	//Pointer to the ROS image 
	sensor_msgs::ImagePtr img_out;

	//Names of the topics 
	std::string sub_topic_1 = "";
	std::string sub_topic_2 = "";
	std::string pub_topic = "";

	//Opencv Mat containter where the images are located
	cv::Mat final_img = cv::Mat::zeros(cv::Size(1280,480),CV_8UC3);
	cv::Mat cam_1 = cv::Mat::zeros(cv::Size(640,480),CV_8UC3);
	cv::Mat cam_2 = cv::Mat::zeros(cv::Size(640,480),CV_8UC3);

	//Counter used for the ROS image
	unsigned long img_counter = 0;

	//Read Parameters
	bool readConfig()
	{
		bool load_param = true;
		if(!n.getParam("cam_1",sub_topic_1))
		{
			ROS_ERROR("\nFailed to load a camera_1 topic\n");
			load_param = false;
		}
		if(!n.getParam("cam_2",sub_topic_2))
		{
			ROS_ERROR("\nFailed to load a camera_2 topic\n");
			load_param = false;
		}
		if(!n.getParam("img_out",pub_topic))
		{
			ROS_ERROR("\nFailed to load output topic\n");
			load_param = false;
		}

		return load_param;
	}

	//Sets the image information if the image is not empty
	void getRGBCameraInfo()
	{
		if(!final_img.empty())
		{
			img_counter++;
			std_msgs::Header header;
			header.seq = img_counter;
			header.stamp = ros::Time::now();
			cv_bridge::CvImage _img;
			_img.encoding = "bgr8";
			_img.image = final_img.clone();
			img_out = _img.toImageMsg();
			img_pub.publish(img_out);
		}
	}

	public:

	//Constructor that subscribes to the camera topics and publishes a new one 
	multiVideo() : it(n)
	{
		if(!readConfig()) ROS_WARN("\nNO PARAMETERS RECEIVED\n");
		img_sub_1 = it.subscribe(sub_topic_1,1,&multiVideo::imageCb_1,this);
		img_sub_2 = it.subscribe(sub_topic_2,1,&multiVideo::imageCb_2,this);
		img_pub = it.advertise(pub_topic,100);
		cv::namedWindow(OPENCV_WINDOW);
	}

	//Destructor that gets rid of the window that shows the images 
	~multiVideo()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	//Loop function that processes and shows the images 
	void update()
	{
		cv::Mat s_1,s_2;
		cv::resize(cam_1,s_1,cv::Size(640,480));
		cv::resize(cam_2,s_2,cv::Size(640,480));
		s_1.copyTo(final_img(cv::Rect(0,0,640,480)));
		s_2.copyTo(final_img(cv::Rect(640,0,640,480)));
		cv::imshow(OPENCV_WINDOW,final_img);
		cv::waitKey(3);
		getRGBCameraInfo();
	}

	//ImageCalback that is called each time a new image is found from cam_!
	void imageCb_1(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr; 
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge (1) exception: %s", e.what());
			return;
		}

		cam_1 = cv_ptr->image;
		update();
	}

	//Image Callback that is called each time a new image is found from cam_2
	void imageCb_2(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr; 
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge (2) exception: %s", e.what());
			return;
		}

		cam_2 = cv_ptr->image;
		update();
	}


};

//Main function
int main(int argc, char* argv[])
{
	ros::init(argc,argv,"multiple_video");
	multiVideo sv;
	ros::spin();
	return 0;
}
