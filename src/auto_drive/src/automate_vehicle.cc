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

cv::Point prev_pt[2];
cv::Point centroid[2];
cv::Point frame_pt;

double threshdistance[2];
std::vector<double> dist[2];
int error;

double PI = atan(1)*4;
bool SHOW_FRAME = true;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv::Mat frame, gray, clipped;

  frame = cv_bridge::toCvShare(msg, "bgr8")->image;
  gray = cv_bridge::toCvShare(msg, "mono8")->image;

  cv::threshold(gray, gray, 160, 255, cv::THRESH_BINARY);

  clipped = gray(cv::Rect(0, gray.rows / 3 * 2, gray.cols, gray.rows / 3));

  cv::Mat labels, stats, centroids;
  int cnt = cv::connectedComponentsWithStats(clipped, labels, stats, centroids);

  // the first componenet is the background layer
  // which will always be present
  if (cnt > 1) {
    for (int i = 1; i < cnt; i++) {
      double *p = centroids.ptr<double>(i);
      for (int j = 0; j < 2; j++) {
        dist[j].push_back(abs(p[0] - prev_pt[j].x));
      }
    }

    for (int i = 0; i < 2; i++) {
      int idx = min_element(dist[i].begin(), dist[i].end()) - dist[i].begin() + 1;
      centroid[i] = cv::Point2d(centroids.at<double>(idx, 0),
                                centroids.at<double>(idx, 1));
      dist[i].clear();
    }
  } else {
    for (int i = 0; i < 2; i++) {
      centroid[i] = prev_pt[i];
    }
  }

  for (int i = 0; i < 2; i++) {
    prev_pt[i] = centroid[i];
  }

  frame_pt.x = (centroid[0].x + centroid[1].x) / 2;
  frame_pt.x = (centroid[0].x + (centroid[1].x - centroid[0].x) / 3);
  frame_pt.y = (centroid[0].y + centroid[1].y) / 2 + gray.rows / 3 * 2;
  cvtColor(clipped, clipped, cv::COLOR_GRAY2BGR);

  cv::circle(frame, frame_pt, 2, cv::Scalar(0, 0, 255), 2);
  cv::circle(clipped, centroid[0], 2, cv::Scalar(0, 255, 0), 2);
  cv::circle(clipped, centroid[1], 2, cv::Scalar(255, 0, 0), 2);

  error = clipped.cols / 2 - frame_pt.x;

  if (SHOW_FRAME) {
    imshow("frame", frame);
    imshow("clipped", clipped);
    cv::waitKey(1);
  }

  geometry_msgs::Twist cmd_vel;
  double x = std::abs(20.0 / error);  // 20 is empirically determined
  if (x > 0.5) {
    x = 0.5;
  }
  if (x < 0.2) {
    x = 0.2;
  }
  cmd_vel.linear.x = x;

  // The unit is rad/s, so the multiplier is dependent on how much
  // the angular speed is to be acted upon in this callback.
  // But we don't know the image publishing rate.
  // 2 seems to give a good result.
  cmd_vel.angular.z = (error*PI/gray.cols) * 2;

  // std::cout << error << " " << cmd_vel.angular.z << " ";
  // std::cout << cmd_vel.linear.x << "\n";

  pub.publish(cmd_vel);
}

int main(int argc, char **argv)
{
  if (argc == 2 && !strcmp(argv[1], "0")) {
    SHOW_FRAME = false;
  }

  prev_pt[0] = cv::Point(250, 60);
  prev_pt[1] = cv::Point(650, 60);

  ros::init(argc, argv, AUTOMATE_VEHICLE);
  ros::NodeHandle nodeHandle;

  image_transport::ImageTransport it(nodeHandle);
  image_transport::Subscriber sub = it.subscribe(TOPIC_IMAGE, 10, imageCallback);
  pub = nodeHandle.advertise<geometry_msgs::Twist>(TOPIC_CMDVEL, 10);

  ros::spin();

  return 0;
}
