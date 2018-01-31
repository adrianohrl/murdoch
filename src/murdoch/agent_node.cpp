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
    bool reauction;
    pnh.param("reauction", reauction, true);
    bool bid_update;
    pnh.param("bid_update", bid_update, false);
    int max_size;
    pnh.param("max_size", max_size, 1);
    role.reset(new talmech::auction::Auctioneer(id,
        nh_, ros::Duration(auction_duration), ros::Rate(renewal_rate),
        sorted_insertion, reauction, bid_update, max_size));
    task_sub_ = nh_->subscribe("/murdoch/task", 10, &AgentNode::taskCallback, this);
  }
  else
  {
    int max_size;
    pnh.param("max_size", max_size, 1);
    role.reset(new talmech::auction::Bidder(id,
        nh_, talmech::auction::MetricsEvaluatorPtr(), max_size));
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

void AgentNode::taskCallback(const talmech_msgs::Task& msg)
{
  talmech::TaskPtr task(new talmech::Task(msg));
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
