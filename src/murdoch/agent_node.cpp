#include "murdoch/agent_node.h"
#include <talmech/auction/auctioneer.h>
#include <talmech/auction/bidder.h>

namespace murdoch
{
AgentNode::AgentNode(const ros::NodeHandlePtr& nh, const ros::Rate& rate)
    : ROSNode::ROSNode(nh, rate)
{
}

AgentNode::~AgentNode() { task_sub_.shutdown(); }

void AgentNode::readParameters()
{
  ros::NodeHandle pnh("~");
  std::string agent_type;
  pnh.param("type", agent_type, std::string("robot"));
  std::string id;
  pnh.param("id", id, std::string(""));
  if (id.empty())
  {
    throw utilities::Exception("The agent id must not be null");
  }
  std::string role_type;
  pnh.param("role", role_type, std::string("bidder"));
  talmech::RolePtr role;
  if (role_type == "auctioneer")
  {
    double auction_duration;
    pnh.param("auction_duration", auction_duration, 1.5);
    double renewal_rate;
    pnh.param("renewal_rate", renewal_rate, 2.0);
    bool sorted_insertion;
    pnh.param("sorted_insertion", sorted_insertion, true);
    bool reallocation;
    pnh.param("reallocation", reallocation, true);
    bool bid_update;
    pnh.param("bid_update", bid_update, false);
    role.reset(new talmech::auction::Auctioneer(nh_, ros::Duration(auction_duration),
                                                ros::Rate(renewal_rate), sorted_insertion,
                                                reallocation, bid_update));
    task_sub_ = nh_->subscribe("task", 10, &AgentNode::taskCallback, this);
  }
  else
  {
    role.reset(new talmech::auction::Bidder(
        nh_, talmech::auction::MetricsEvaluatorPtr()));
  }
  if (agent_type == "agent")
  {
    agent_.reset(new talmech::Agent(id, role));
  }
  else
  {
    agent_.reset(new talmech::Robot(id, role));
  }
}

void AgentNode::taskCallback(const std_msgs::String& msg)
{
  talmech::TaskPtr task(new talmech::Task(msg.data));
  talmech::auction::AuctioneerPtr auctioneer(
      boost::dynamic_pointer_cast<talmech::auction::Auctioneer>(
          agent_->getRole()));
  if (auctioneer->auction(task))
  {
    ROS_INFO_STREAM("[AgentNode] Auctioning " << *task << "...");
  }
  else
  {
    ROS_WARN_STREAM("[AgentNode] Unable to auction " << *task << "...");
  }
}
}
