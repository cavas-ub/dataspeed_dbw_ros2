#include <dataspeed_dbw_msgs/msg/brake_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/brake_report.hpp>
#include <dataspeed_dbw_msgs/msg/gear_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/gear_report.hpp>
#include <dataspeed_dbw_msgs/msg/steering_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/steering_report.hpp>
#include <dataspeed_dbw_msgs/msg/throttle_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/throttle_report.hpp>
// you cannot use the #include directive in a macro
#include <dbw_ford_msgs/msg/brake_cmd.hpp>
#include <dbw_ford_msgs/msg/brake_report.hpp>
#include <dbw_ford_msgs/msg/gear_cmd.hpp>
#include <dbw_ford_msgs/msg/gear_report.hpp>
#include <dbw_ford_msgs/msg/steering_cmd.hpp>
#include <dbw_ford_msgs/msg/steering_report.hpp>
#include <dbw_ford_msgs/msg/throttle_cmd.hpp>
#include <dbw_ford_msgs/msg/throttle_report.hpp>

#include <rclcpp/rclcpp.hpp>

// #include "message_copy.hpp"
#include "generic_message.hpp"

namespace dataspeed_dbw_gateway {

namespace common_ns = dataspeed_dbw_msgs::msg;
namespace vehicle_ns = dbw_ford_msgs::msg;

// Message Name, Message Topic
#define MESSAGE_LIST_N(X0, X1, X2, X3, X4, X5, X6, X7) \
  X0(BrakeCmd, brake_cmd)                              \
  X1(BrakeReport, brake_report)                        \
  X2(GearCmd, gear_cmd)                                \
  X3(GearReport, gear_report)                          \
  X4(SteeringCmd, steering_cmd)                        \
  X5(SteeringReport, steering_report)                  \
  X6(ThrottleCmd, throttle_cmd)                        \
  X7(ThrottleReport, throttle_report)

#define MESSAGE_LIST(X) MESSAGE_LIST_N(X, X, X, X, X, X, X, X)
#define EMPTY(xn, xt)

class FordGateway : public rclcpp::Node {
 public:
  FordGateway(const rclcpp::NodeOptions &options) : rclcpp::Node("gateway", options) {
    using std::placeholders::_1;

    auto subOptions = rclcpp::SubscriptionOptions();
    subOptions.ignore_local_publications = true;


    {
      // vehicle publishers/subscribers
      auto node = create_sub_node("vehicle");

#define INIT_VEHICLE_PUB_SUB(xname, xtopic)                                            \
  pubVeh##xname = node->create_publisher<vehicle_ns::xname>(#xtopic, rclcpp::QoS(10)); \
  subVeh##xname = node->create_subscription<vehicle_ns::xname>(                        \
      #xtopic, rclcpp::QoS(10),                                                        \
      [this](vehicle_ns::xname::ConstSharedPtr msg) { onMessage(this->pubCommon##xname, msg); }, subOptions);

      MESSAGE_LIST(INIT_VEHICLE_PUB_SUB)
    }
    {
      // common publishers/subscribers
      auto node = create_sub_node("ds");

#define INIT_COMMON_PUB_SUB(xname, xtopic)                                               \
  pubCommon##xname = node->create_publisher<common_ns::xname>(#xtopic, rclcpp::QoS(10)); \
  subCommon##xname = node->create_subscription<common_ns::xname>(                        \
      #xtopic, rclcpp::QoS(10),                                                          \
      [this](common_ns::xname::ConstSharedPtr msg) { onMessage(this->pubVeh##xname, msg); }, subOptions);

      MESSAGE_LIST(INIT_COMMON_PUB_SUB)
    }
  }
  template <typename In, typename Out>
  void onMessage(std::shared_ptr<rclcpp::Publisher<Out>> pub, const std::shared_ptr<const In> msg) {
    auto out = std::make_unique<Out>();
    // dataspeed_message_inspector::MessageInspector<In> min(msg.get());
    // dataspeed_message_inspector::MessageInspector<Out> mout(out.get());
    // min.copy_to(&mout);
    // dataspeed_message_copy::copy_message(msg, out.get());
    ros2_generic_message::Message<In> msgIn(msg.get());
    ros2_generic_message::Message<Out> msgOut(out.get());
    msgOut.set(msgIn);
    
    pub->publish(std::move(out));
  }

 private:
#define DECLARE_PUB_SUB(xname, xtopic)                              \
  rclcpp::Publisher<vehicle_ns::xname>::SharedPtr pubVeh##xname;    \
  rclcpp::Publisher<common_ns::xname>::SharedPtr pubCommon##xname;  \
  rclcpp::Subscription<vehicle_ns::xname>::SharedPtr subVeh##xname; \
  rclcpp::Subscription<common_ns::xname>::SharedPtr subCommon##xname;
  MESSAGE_LIST(DECLARE_PUB_SUB)
};

}  // namespace dataspeed_dbw_gateway

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(dataspeed_dbw_gateway::FordGateway)
