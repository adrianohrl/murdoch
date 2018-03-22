#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <talmech_msgs/Task.h>

typedef boost::shared_ptr<ros::Publisher> Publisher;
typedef std::map<std::string, Publisher> Publishers;
typedef Publishers::iterator PublishersIt;
typedef Publishers::const_iterator PublishersConstIt;
typedef geometry_msgs::PoseStamped Pose;
typedef std::vector<Pose> Poses;
typedef Poses::iterator PosesIt;
typedef Poses::const_iterator PosesConstIt;
typedef std::vector<talmech_msgs::Feature> features;
typedef features::iterator featuresIt;
typedef features::const_iterator featuresConstIt;

int counter = 0;
ros::Time timestamp_;
ros::NodeHandlePtr nh_;
ros::Subscriber subscriber_;
ros::Publisher cycles_pub_;
ros::Publisher waypoints_pub_;
ros::Publisher waypoints_x_pub_;
ros::Publisher waypoints_y_pub_;
Publishers features_pub_;
Publishers features_level_pub_;

void shutdown()
{
  subscriber_.shutdown();
  cycles_pub_.shutdown();
  waypoints_pub_.shutdown();
  waypoints_x_pub_.shutdown();
  waypoints_y_pub_.shutdown();
  for (PublishersIt it(features_pub_.begin()); it != features_pub_.end(); it++)
  {
    it->second->shutdown();
  }
  for (PublishersIt it(features_level_pub_.begin()); it != features_level_pub_.end(); it++)
  {
    it->second->shutdown();
  }
}

void callback(const talmech_msgs::Task::ConstPtr& msg)
{
  counter++;
  ROS_INFO_STREAM("Received " << msg->id << " (counter: " << counter << ")");
  ros::Time timestamp(ros::Time::now());
  std_msgs::Float64 double_msg;
  double_msg.data = (timestamp - timestamp_).toSec();
  ROS_INFO_STREAM("cycles_pub_ publishing: " << double_msg.data);
  cycles_pub_.publish(double_msg);
  std_msgs::Int64 int_msg;
  int_msg.data = msg->waypoints.poses.size();
  ROS_INFO_STREAM("waypoints_pub_ publishing: " << int_msg.data);
  waypoints_pub_.publish(int_msg);
  for (PosesConstIt it(msg->waypoints.poses.begin()); it != msg->waypoints.poses.end(); it++)
  {
    double_msg.data = it->pose.position.x;
    ROS_INFO_STREAM("waypoints_x_pub_ publishing: " << double_msg.data);
    waypoints_x_pub_.publish(double_msg);
    double_msg.data = it->pose.position.y;
    ROS_INFO_STREAM("waypoints_y_pub_ publishing: " << double_msg.data);
    waypoints_y_pub_.publish(double_msg);
  }
  std_msgs::Bool bool_msgs;
  bool_msgs.data = true;
  for (featuresConstIt it(msg->features.begin()); it != msg->features.end(); it++)
  {
    PublishersIt pit(features_pub_.find(it->resource));
    if (pit == features_pub_.end())
    {
      Publisher publisher(new ros::Publisher(nh_->advertise<std_msgs::Bool>("/analytics/features/" + it->resource, 1000)));
      std::pair<std::string, Publisher> features_pub(it->resource, publisher);
      features_pub_.insert(features_pub);
      pit = features_pub_.find(it->resource);
      if (it->type != 0)
      {
        publisher = Publisher(new ros::Publisher(nh_->advertise<std_msgs::Float64>("/analytics/features/" + it->resource + "/level", 1000)));
        std::pair<std::string, Publisher> features_level_pub(it->resource, publisher);
        features_level_pub_.insert(features_level_pub);
      }
    }
    ROS_INFO_STREAM("features_pub_ publishing: " << (bool_msgs.data ? "true" : "false"));
    pit->second->publish(bool_msgs);
    if (it->type != 0)
    {
      PublishersIt plit(features_level_pub_.find(it->resource));
      double_msg.data = it->level;
      ROS_INFO_STREAM("features_level_pub_ publishing: " << double_msg.data);
      plit->second->publish(double_msg);
    }
  }
  timestamp_ = timestamp;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "analytics_node");
  nh_ = ros::NodeHandlePtr(new ros::NodeHandle());
  timestamp_ = ros::Time::now();
  subscriber_ = nh_->subscribe("/murdoch/task", 1000, callback);
  cycles_pub_ = nh_->advertise<std_msgs::Float64>("/analytics/cycles", 1000);
  waypoints_pub_ = nh_->advertise<std_msgs::Int64>("/analytics/waypoints", 1000);
  waypoints_x_pub_ = nh_->advertise<std_msgs::Float64>("/analytics/waypoints/x", 1000);
  waypoints_y_pub_ = nh_->advertise<std_msgs::Float64>("/analytics/waypoints/y", 1000);
  ros::spin();
  shutdown();
  return 0;
}
