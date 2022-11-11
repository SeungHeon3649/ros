#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <memory>
#include <thread>
#include <functional>
#include <atomic>
using namespace cv;

namespace jetson_camera
{

class JetsonCameraNodelet : public nodelet::Nodelet
{
public:
	~JetsonCameraNodelet()
	{
		is_running_ = false;
		cap_thread_->join();
	}

private:
	static std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
		return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
		       std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
		       "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
		       std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
	}

	ros::NodeHandle nh_, nh_priv_;
	image_transport::CameraPublisher pub_;
	image_transport::Subscriber copy_sub; //+
	image_transport::Publisher copy_pub; //+
	cv_bridge::CvImagePtr cv_ptr;//+
	sensor_msgs::ImageConstPtr copy_msg;//+
	//Mat dst;

	int cap_width_, cap_height_;
	int width_, height_, framerate_;
	int flip_method_;
	std::string frame_id_;
	double delay_;
	sensor_msgs::CameraInfo cam_info_;

	std::shared_ptr<std::thread> cap_thread_;
	std::atomic<bool> is_running_;

	void onInit() override
	{
		nh_ = getNodeHandle();
		nh_priv_ = getPrivateNodeHandle();

		image_transport::ImageTransport it(nh_);
		image_transport::ImageTransport it_priv(nh_priv_);

		cap_width_ = nh_priv_.param<int>("cap_width", 1280);
		cap_height_ = nh_priv_.param<int>("cap_height", 720);

		width_ = nh_priv_.param<int>("width", 640);
		height_ = nh_priv_.param<int>("height", 480);
		framerate_ = nh_priv_.param<int>("fps", 60);

		flip_method_ = nh_priv_.param<int>("flip_method", 0);

		frame_id_ = nh_priv_.param<std::string>("frame_id", "main_camera_optical");
		delay_ = nh_priv_.param<double>("capture_delay", 0.0);

		std::string topic_name = nh_priv_.param<std::string>("topic_name", "image_raw");

		std::string url = nh_priv_.param<std::string>("camera_info_url", "");

		camera_info_manager::CameraInfoManager info_manager(nh_, "camera", url);

		cam_info_ = info_manager.getCameraInfo();
		cam_info_.header.frame_id = frame_id_;

		// Rescale camera information

		double width_coeff = static_cast<double>(width_) / cam_info_.width;
		double height_coeff = static_cast<double>(height_) / cam_info_.height;

		cam_info_.width = width_;
		cam_info_.height = height_;

		cam_info_.K[0] *= width_coeff;
		cam_info_.K[2] *= width_coeff;
		cam_info_.K[4] *= height_coeff;
		cam_info_.K[5] *= height_coeff;

		cam_info_.P[0] *= width_coeff;
		cam_info_.P[2] *= width_coeff;
		cam_info_.P[5] *= height_coeff;
		cam_info_.P[6] *= height_coeff;

		NODELET_INFO("jetson_camera: Starting node with the following parameters:\n"\
		             "capture width: %d, capture height: %d\n"\
		             "published width: %d, published height: %d\n"\
		             "requested framerate: %d, flip method: %d\n",
		             cap_width_, cap_height_, width_, height_, framerate_, flip_method_);

		pub_ = it_priv.advertiseCamera(topic_name, 1);

		is_running_ = true;
		cap_thread_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&JetsonCameraNodelet::captureFunc, this)));
	}

	void copyCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		//cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		copy_msg = msg;
	}

	void captureFunc()
	{
		std::string pipeline = gstreamer_pipeline(cap_width_, cap_height_, width_, height_, framerate_, flip_method_);
		NODELET_INFO("Pipeline: %s", pipeline.c_str());
		cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
		cv_bridge::CvImage img;
		image_transport::ImageTransport it_(nh_); //+
		copy_sub = it_.subscribe("/main_camera/image_raw", 1, &JetsonCameraNodelet::copyCallback, this); //+
		copy_pub = it_.advertise("copy_img", 1); //+
		//Mat copy; //+
		img.encoding = sensor_msgs::image_encodings::BGR8;
		img.header.frame_id = frame_id_;

		while (is_running_.load())
		{
			if (cap.read(img.image))
			{
				img.header.stamp = ros::Time::now() - ros::Duration(delay_);
				cam_info_.header.stamp = img.header.stamp;
				pub_.publish(*img.toImageMsg(), cam_info_);
				// putText(cv_ptr->image, "copy", Point(50, 50), 
				// FONT_HERSHEY_SIMPLEX, 2.5, Scalar(255, 255, 255), 3);
				copy_pub.publish(copy_msg);
			}
		}
	}

};

}

PLUGINLIB_EXPORT_CLASS(jetson_camera::JetsonCameraNodelet, nodelet::Nodelet)
