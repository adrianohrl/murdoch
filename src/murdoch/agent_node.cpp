#include "murdoch/agent_node.h"
#include <talmech/auction/auctioneer.h>
#include <talmech/auction/bidder.h>

namespace murdoch
{
AgentNode::AgentNode(const ros::NodeHandlePtr& nh, const ros::Rate& rate)
    : ROSNode::ROSNode(nh, rate)
{
}

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
    role.reset(new talmech::auction::Auctioneer(nh_));
  }
  else
  {
    role.reset(new talmech::auction::Bidder(nh_, talmech::auction::MetricsEvaluatorPtr()));
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
}
