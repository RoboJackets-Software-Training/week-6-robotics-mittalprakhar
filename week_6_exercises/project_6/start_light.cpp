#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

ros::Publisher debug_red_pub, debug_green_pub, start_pub;

std_msgs::Bool start_msg;
ros::Time red_time;

double min_circularity_, max_circularity_, min_area_;

cv::Mat kernel(int x, int y)
{
    return cv::getStructuringElement(cv::MORPH_RECT, cv::Size(x, y));
}

bool is_circle(cv::Mat img)
{
	std::vector<std::vector<cv::Point>> contours;
    findContours(img, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    for (const auto &contour : contours)
    {
        double perimeter = cv::arcLength(contour, true);
        double area = cv::contourArea(contour, false);
        double circularity = 4 * 3.1415926 * (area / (perimeter * perimeter));

        if (min_circularity_ < circularity && circularity < max_circularity_ && area > min_area_)
        {
            return true;
        }
    }
    return false;
}

void camera_callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    cv::Mat hsv_frame, red_pixels, green_pixels;
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

    cv::inRange(hsv_frame, cv::Scalar(0, 110, 110), cv::Scalar(15, 255, 255), red_pixels);
    cv::inRange(hsv_frame, cv::Scalar(40, 120, 120), cv::Scalar(100, 255, 255), green_pixels);

    cv::morphologyEx(red_pixels, red_pixels, cv::MORPH_OPEN, kernel(5, 5));
    cv::morphologyEx(green_pixels, green_pixels, cv::MORPH_OPEN, kernel(5, 5));

    bool red_circle = is_circle(red_pixels);
    bool green_circle = is_circle(green_pixels);

  	ros::Time curr_time = msg->header.stamp;

    if (red_circle)
    {
        red_time = curr_time;
    }

    start_msg.data = green_circle && (curr_time - red_time).toSec() < 1;
    start_pub.publish(start_msg);

    sensor_msgs::Image out;

    cv_ptr->image = red_pixels;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(out);
    debug_red_pub.publish(out);

    cv_ptr->image = green_pixels;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(out);
    debug_green_pub.publish(out);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "start_light");
	ros::NodeHandle pnh("~");

	pnh.getParam("min_circularity", min_circularity_);
    pnh.getParam("max_circularity", max_circularity_);
    pnh.getParam("min_area_", min_area_);
		
	ros::Subscriber img_sub = pnh.subscribe("/camera/image", 1, camera_callback);

    debug_red_pub = pnh.advertise<sensor_msgs::Image>("debug/red", 1);
    debug_green_pub = pnh.advertise<sensor_msgs::Image>("debug/green", 1);

	start_pub = pnh.advertise<std_msgs::Bool>("/event/race_started", 1);

	ros::spin();
}