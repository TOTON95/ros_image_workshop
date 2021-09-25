/**
 * multi_camera_overlay.cpp
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

//Viewer name 
static const std::string OPENCV_WINDOW = "Multi camera overlay viewer";

//Multivideo object 
class multiVideo
{
	//ROS Node handle and image transport component attached
	ros::NodeHandle n;
	image_transport::ImageTransport it;
	image_transport::Subscriber img_sub_1;
	image_transport::Subscriber img_sub_2;
	image_transport::Publisher img_pub;
	image_transport::Publisher overlay_pub_1;
	image_transport::Publisher overlay_pub_2;

	//ROS image pointers
	sensor_msgs::ImagePtr img_out;
	sensor_msgs::ImagePtr overlay_1_out;
	sensor_msgs::ImagePtr overlay_2_out;

	//Topic names
	std::string sub_topic_1 = "";
	std::string sub_topic_2 = "";
	std::string pub_topic = "";
	std::string pub_overlay_1 = "";
	std::string pub_overlay_2 = "";

	//OpenCV images
	cv::Mat final_img = cv::Mat::zeros(cv::Size(1280,960),CV_8UC3);
	cv::Mat cam_1 = cv::Mat::zeros(cv::Size(640,480),CV_8UC3);
	cv::Mat cam_2 = cv::Mat::zeros(cv::Size(640,480),CV_8UC3);
	cv::Mat overlay_1 = cv::Mat::zeros(cv::Size(640,480),CV_8UC3);
	cv::Mat overlay_2 = cv::Mat::zeros(cv::Size(640,480),CV_8UC3);

	//Image Counter (needed to publish the ROS images)
	unsigned long img_counter, overlay1_counter, overlay2_counter = 0;

	//Get parameters values
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
		if(!n.getParam("overlay_1",pub_overlay_1))
		{
			ROS_ERROR("\nFailed to load overlay_1 topic\n");
			load_param = false;
		}
		if(!n.getParam("overlay_2",pub_overlay_2))
		{
			ROS_ERROR("\nFailed to load overlay_2 topic\n");
			load_param = false;
		}

		return load_param;
	}

	//Assemble and publish ROS images
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

	//
	void getOverlayInfo1()
	{
		if(!overlay_1.empty())
		{
			overlay1_counter++;
			std_msgs::Header header;
			header.seq = overlay1_counter;
			header.stamp = ros::Time::now();
			cv_bridge::CvImage _img;
			_img.encoding = "bgr8";
			_img.image = overlay_1.clone();
			overlay_1_out = _img.toImageMsg();
			overlay_pub_1.publish(overlay_1_out);
		}
	}

	void getOverlayInfo2()
	{
		if(!overlay_2.empty())
		{
			overlay2_counter++;
			std_msgs::Header header;
			header.seq = overlay2_counter;
			header.stamp = ros::Time::now();
			cv_bridge::CvImage _img;
			_img.encoding = "bgr8";
			_img.image = overlay_2.clone();
			overlay_2_out = _img.toImageMsg();
			overlay_pub_2.publish(overlay_2_out);
		}
	}

	void writeOverlay()
	{
		if(!overlay_1.empty())
		{
			cv::line(overlay_1, cv::Point(0,0),cv::Point(640,480),cv::Scalar(0,255,0),2,cv::LINE_AA);
			cv::line(overlay_1, cv::Point(640,0),cv::Point(0,480),cv::Scalar(0,255,0),2,cv::LINE_AA);
		}
		if(!overlay_2.empty())
		{
			cv::circle(overlay_2,cv::Point(320,240),5,cv::Scalar(0,0,255),-1,cv::LINE_AA);
			cv::putText(overlay_2,"TEST", cv::Point(440,400),cv::FONT_HERSHEY_SIMPLEX,2.0,cv::Scalar(0,255,0),2);
		}
	}

	public:

	multiVideo() : it(n)
	{
		if(!readConfig()) ROS_WARN("\nNO PARAMETERS RECEIVED\n");
		img_sub_1 = it.subscribe(sub_topic_1,1,&multiVideo::imageCb_1,this);
		img_sub_2 = it.subscribe(sub_topic_2,1,&multiVideo::imageCb_2,this);
		img_pub = it.advertise(pub_topic,100);
		overlay_pub_1 = it.advertise(pub_overlay_1,100);
		overlay_pub_2 = it.advertise(pub_overlay_2,100);
		cv::namedWindow(OPENCV_WINDOW);
	}

	~multiVideo()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void update()
	{
		cv::Mat s_1,s_2,s_3,s_4;
		cv::resize(cam_1,s_1,cv::Size(640,480));
		cv::resize(cam_2,s_2,cv::Size(640,480));
		cv::resize(overlay_1,s_3,cv::Size(640,480));
		cv::resize(overlay_2,s_4,cv::Size(640,480));
		s_1.copyTo(final_img(cv::Rect(0,0,640,480)));
		s_2.copyTo(final_img(cv::Rect(640,0,640,480)));
		s_3.copyTo(final_img(cv::Rect(0,480,640,480)));
		s_4.copyTo(final_img(cv::Rect(640,480,640,480)));
		cv::imshow(OPENCV_WINDOW,final_img);
		cv::waitKey(3);
		getRGBCameraInfo();
		getOverlayInfo1();
		getOverlayInfo2();
	}

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
		overlay_1 = cam_1.clone();
		writeOverlay();
		update();
	}

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
		overlay_2 = cam_2.clone();
		writeOverlay();
		update();
	}


};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"single_video");
	multiVideo sv;
	ros::spin();
	return 0;
}
