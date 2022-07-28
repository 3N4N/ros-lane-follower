#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>


const std::string AUTOMATE_VEHICLE = "node_automate_vehicle";
const std::string TOPIC_IMAGE = "/vehicle_camera/image_raw";

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv::Mat frame, gray, dst;

  frame = cv_bridge::toCvShare(msg, "bgr8")->image;
  gray = cv_bridge::toCvShare(msg, "mono8")->image;

  gray = gray + 100 - mean(gray)[0];
  threshold(gray, gray, 160, 255, cv::THRESH_BINARY);

  dst = gray(cv::Rect(0, gray.rows / 3 * 2, gray.cols, gray.rows / 3));

  cv::imwrite("frame.png", frame);
  cv::imwrite("gray.png", gray);
  cv::imwrite("dst.png", dst);

  cv::Mat labels, stats, centroids;
  int cnt = connectedComponentsWithStats(dst, labels, stats, centroids);

  std::cout << cnt << " ";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, AUTOMATE_VEHICLE);
  ros::NodeHandle nodeHandle;

  image_transport::ImageTransport it(nodeHandle);
  image_transport::Subscriber sub = it.subscribe(TOPIC_IMAGE, 100, imageCallback);

  ros::spin();

  return 0;
}
