#ifndef _MURDOCH_AUCTIONEER_NODE_H_
#define _MURDOCH_AUCTIONEER_NODE_H_

#include <boost/shared_ptr.hpp>
#include "utilities/ros_node.h"

namespace murdoch
{
class AuctioneerNode : public utilities::ROSNode
{
public:
  typedef boost::shared_ptr<AuctioneerNode> Ptr;
  typedef boost::shared_ptr<const AuctioneerNode> ConstPtr;
  AuctioneerNode(const ros::NodeHandlePtr& nh,
                 const ros::Rate& rate = ros::Rate(20));
private:
  virtual void controlLoop();
};
typedef AuctioneerNode::Ptr AuctioneerNodePtr;
typedef AuctioneerNode::ConstPtr AuctioneerNodeConstPtr;
}

#endif // _MURDOCH_AUCTIONEER_NODE_H_
