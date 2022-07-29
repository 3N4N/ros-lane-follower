#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <geometry_msgs/Twist.h>

const std::string AUTOMATE_VEHICLE = "node_automate_vehicle";
const std::string TOPIC_IMAGE = "/vehicle_camera/image_raw";
const std::string TOPIC_CMDVEL = "vehicle/cmd_vel";

ros::Publisher pub;

cv::Point prevpt1;
cv::Point prevpt2;
cv::Point centroid[2];
cv::Point fpt;
int minlb[2];
double threshdistance[2];
std::vector<double> dis1;
std::vector<double> dis2;
int error;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv::Mat frame, gray, dst;

  frame = cv_bridge::toCvShare(msg, "bgr8")->image;
  gray = cv_bridge::toCvShare(msg, "mono8")->image;

  // gray = gray + 100 - mean(gray)[0];
  threshold(gray, gray, 160, 255, cv::THRESH_BINARY);

  dst = gray(cv::Rect(0, gray.rows / 3 * 2, gray.cols, gray.rows / 3));

  // cv::imwrite("dst.png", dst);

  cv::Mat labels, stats, centroids;
  int cnt = connectedComponentsWithStats(dst, labels, stats, centroids);

  // std::cout << cnt << " ";
  // std::cout << "\n" << labels << "\n";
  // std::cout << "\n" << stats << "\n";
  // std::cout << "\n" << centroids << "\n\n\n";

  // the first componenet is the background layer
  // which will always be present
  if (cnt > 1) {
    for (int i = 1; i < cnt; i++) {
      double *p = centroids.ptr<double>(i);
      dis1.push_back(abs(p[0] - prevpt1.x));
      dis2.push_back(abs(p[0] - prevpt2.x));
    }

    threshdistance[0] = *min_element(dis1.begin(), dis1.end());
    threshdistance[1] = *min_element(dis2.begin(), dis2.end());

    // don't consider lines too far away
    // most likely they are not the one we were following

    if (threshdistance[0] > 100) {
      centroid[0] = prevpt1;
    } else {
      minlb[0] = min_element(dis1.begin(), dis1.end()) - dis1.begin();
      centroid[0] = cv::Point2d(centroids.at<double>(minlb[0] + 1, 0),
                                centroids.at<double>(minlb[0] + 1, 1));
    }

    if (threshdistance[1] > 100) {
      centroid[1] = prevpt2;
    } else {
      minlb[1] = min_element(dis2.begin(), dis2.end()) - dis2.begin();
      centroid[1] = cv::Point2d(centroids.at<double>(minlb[1] + 1, 0),
                                centroids.at<double>(minlb[1] + 1, 1));
    }

    dis1.clear();
    dis2.clear();
  } else {
    centroid[0] = prevpt1;
    centroid[1] = prevpt2;
  }

  prevpt1 = centroid[0];
  prevpt2 = centroid[1];

  fpt.x = (centroid[0].x + centroid[1].x) / 2;
  fpt.y = (centroid[0].y + centroid[1].y) / 2 + gray.rows / 3 * 2;
  cvtColor(dst, dst, cv::COLOR_GRAY2BGR);

  cv::circle(frame, fpt, 2, cv::Scalar(0, 0, 255), 2);
  cv::circle(dst, centroid[0], 2, cv::Scalar(0, 255, 0), 2);
  cv::circle(dst, centroid[1], 2, cv::Scalar(255, 0, 0), 2);

  error = dst.cols / 2 - fpt.x;

  imshow("frame", frame);
  imshow("dst", dst);
  cv::waitKey(1);

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.5;
  cmd_vel.angular.z = (error*90.0/400)/15;

  pub.publish(cmd_vel);
}

int main(int argc, char **argv)
{
  prevpt1 = cv::Point(110, 60);
  prevpt2 = cv::Point(620, 60);

  ros::init(argc, argv, AUTOMATE_VEHICLE);
  ros::NodeHandle nodeHandle;

  image_transport::ImageTransport it(nodeHandle);
  image_transport::Subscriber sub = it.subscribe(TOPIC_IMAGE, 10, imageCallback);
  pub = nodeHandle.advertise<geometry_msgs::Twist>(TOPIC_CMDVEL, 10);

  ros::spin();

  return 0;
}
