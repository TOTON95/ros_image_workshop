/**
 * simple_camera.cpp
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

//Window's name
static const std::string OPENCV_WINDOW = "Camera viewer";

//Class definition
class singleVideo
{
	//NodeHandler and attached image_transport
	ros::NodeHandle n;
	image_transport::ImageTransport it;
	image_transport::Subscriber img_sub;
	
	//Name of the used topic 
	std::string sub_topic = "";

	//Read the parameters
	bool readConfig()
	{
		bool load_param = true;
		if(!n.getParam("cam_1",sub_topic))
		{
			ROS_ERROR("\nFailed to load a camera topic\n");
			load_param = false;
		}

		return load_param;

	}

	public:

	//Constructor that uses the image_transport to start receiving images
	singleVideo() : it(n)
	{
		if(!readConfig()) ROS_WARN("\nNO PARAMETERS RECEIVED\n");
		img_sub = it.subscribe(sub_topic,1,&singleVideo::imageCb,this);
		cv::namedWindow(OPENCV_WINDOW);
	}

	//Destructor that gets rid of OpenCV window once the process is about to finish
	~singleVideo()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	//ImageCallback used to acquire the image, process it, and show it
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr; 
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}


		cv::imshow(OPENCV_WINDOW,cv_ptr->image);
		cv::waitKey(3);
	}
};


//Main function 
int main(int argc, char* argv[])
{
	ros::init(argc,argv,"single_video");
	singleVideo sv;
	ros::spin();
	return 0;
}
