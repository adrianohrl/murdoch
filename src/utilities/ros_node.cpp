/**
 *  This source file implements the ROSNode class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/ros_node.h"

namespace utilities
{
/**
 * @brief ROSNode::Node builds an ROSNode object given a ROS NodeHandle
 * and also its desired spin rate.
 * @param nh must NOT be NULL.
 * @param rate must be positive.
 */
ROSNode::ROSNode(const ros::NodeHandlePtr& nh, const ros::Rate& rate)
    : nh_(nh), rate_(rate), name_(ros::this_node::getName())
{
  if (!nh)
  {
    ROS_FATAL("ROS node handle must not be NULL!!!");
    ros::shutdown();
    return;
  }
}

/**
 * @brief ROSNode::~ROSNode announces that this ROS node will shutdown. This
 * will not destruct the used ROS NodeHandle object.
 */
ROSNode::~ROSNode() {}

/**
 * @brief ROSNode::run loops while there is not another instance
 * of this node with this node name, or while the Ctrl+C buttons
 * is not pressed at the terminal. In addition, it periodicly updates
 * this node, as well as, controls the updates rate.
 */
void ROSNode::run()
{
  ROS_INFO_STREAM("   Reading " << *this << " parameters ...");
  try
  {
    readParameters();
  }
  catch (const utilities::Exception& ex)
  {
    return;
  }
  ROS_INFO_STREAM_COND(!isSettedUp(), "   Setting " << *this << " up ...");
  while (nh_->ok() && !isSettedUp())
  {
    ros::spinOnce();
    rate_.sleep();
  }
  init();
  ROS_INFO_STREAM(*this << " is up and running.");
  while (nh_->ok())
  {
    controlLoop();
    ros::spinOnce();
    rate_.sleep();
  }
}

/**
 * @brief ROSNode::shutdown
 * @param message
 */
void ROSNode::shutdown(std::string message) const
{
  ROS_WARN_STREAM("   Shutting " << *this << " off ...");
  nh_->shutdown();
  if (!message.empty())
  {
    throw utilities::Exception(message);
  }
}

/**
 * @brief ROSNode::reset
 */
void ROSNode::reset()
{
  ROS_INFO_STREAM("   Resetting " << *this << " ...");
  readParameters();
}

/**
 * @brief ROSNode::setParameters
 */
void ROSNode::readParameters()
{
  ros::NodeHandle pnh("~");
  double rate;
  pnh.param("spin_rate", rate, 30.0);
  rate_ = ros::Rate(rate > 0.0 ? rate : 30.0);
}

/**
 * @brief ROSNode::isSetted
 * @return
 */
bool ROSNode::isSettedUp() { return true; }

/**
 * @brief ROSNode::init
 */
void ROSNode::init() {}

/**
 * @brief ROSNode::getNodeHandle encapsulates this ROS node handle.
 * @return a pointer to an internal member that handles this node.
 */
ros::NodeHandlePtr ROSNode::getNodeHandle() const { return nh_; }

/**
 * @brief ROSNode::getName encapsulates this ROS node name.
 * @return this ROS node whole name.
 */
std::string ROSNode::getName() const { return name_; }

/**
 * @brief ROSNode::ok
 * @return if this ROS node controller is still running properly.
 */
bool ROSNode::ok() const { return nh_->ok(); }

/**
 * @brief operator <<
 * @param out
 * @param node
 * @return
 */
std::ostream& operator<<(std::ostream& out, const ROSNode& node)
{
  out << node.name_;
  return out;
}
}
