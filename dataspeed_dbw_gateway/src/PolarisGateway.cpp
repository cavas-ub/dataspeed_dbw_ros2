#include <dataspeed_dbw_msgs/msg/brake_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/brake_report.hpp>
#include <dataspeed_dbw_msgs/msg/gear_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/gear_report.hpp>
#include <dataspeed_dbw_msgs/msg/steering_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/steering_report.hpp>
#include <dataspeed_dbw_msgs/msg/throttle_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/throttle_report.hpp>

#include <dbw_polaris_msgs/msg/brake_cmd.hpp>
#include <dbw_polaris_msgs/msg/brake_report.hpp>
#include <dbw_polaris_msgs/msg/gear_cmd.hpp>
#include <dbw_polaris_msgs/msg/gear_report.hpp>
#include <dbw_polaris_msgs/msg/steering_cmd.hpp>
#include <dbw_polaris_msgs/msg/steering_report.hpp>
#include <dbw_polaris_msgs/msg/throttle_cmd.hpp>
#include <dbw_polaris_msgs/msg/throttle_report.hpp>

#include <rclcpp/rclcpp.hpp>

#include "generic_message.hpp"

namespace dataspeed_dbw_gateway {

namespace common_ns = dataspeed_dbw_msgs::msg;
namespace vehicle_ns = dbw_polaris_msgs::msg;

// Message Name, Message Topic
#define MESSAGE_LIST_N(X0, X1, X2, X3, X4, X5, X6, X7) \
  X0(BrakeCmd,       brake_cmd)                        \
  X1(BrakeReport,    brake_report)                     \
  X2(GearCmd,        gear_cmd)                         \
  X3(GearReport,     gear_report)                      \
  X4(SteeringCmd,    steering_cmd)                     \
  X5(SteeringReport, steering_report)                  \
  X6(ThrottleCmd,    throttle_cmd)                     \
  X7(ThrottleReport, throttle_report)                  \

#define EMPTY(xn, xt)
#define MESSAGE_LIST(C,R)   MESSAGE_LIST_N(C, R, C, R, C, R, C, R)
#define MESSAGE_LIST_CMD(X) MESSAGE_LIST(X, EMPTY)
#define MESSAGE_LIST_RPT(X) MESSAGE_LIST(EMPTY, X)

/*
 * Forward messages from topic `ds/cmd` to `vehicle/cmd` and convert types
 * Forward messages from topic `vehicle/report` to `ds/report` and convert types
 */
class PolarisGateway : public rclcpp::Node {
public:
  PolarisGateway(const rclcpp::NodeOptions &options) : rclcpp::Node("gateway", options) {
    // QOS options
    const auto QOS = rclcpp::QoS(2);

    // Node namespaces
    auto node_vh = this; // Launch in the vehicle namespace
    auto node_ds = create_sub_node("ds");

    // Publish and subscribe
    #define CMD_PUB_SUB(xname, xtopic) \
    pub_vh_##xtopic = node_vh->create_publisher  <vehicle_ns::xname>(#xtopic, QOS); \
    sub_ds_##xtopic = node_ds->create_subscription<common_ns::xname>(#xtopic, QOS, \
        [this](common_ns::xname::ConstSharedPtr msg) { onMessage(this->pub_vh_##xtopic, msg); });
    #define RPT_PUB_SUB(xname, xtopic) \
    pub_ds_##xtopic = node_ds->create_publisher    <common_ns::xname>(#xtopic, QOS); \
    sub_vh_##xtopic = node_vh->create_subscription<vehicle_ns::xname>(#xtopic, QOS, \
        [this](vehicle_ns::xname::ConstSharedPtr msg) { onMessage(this->pub_ds_##xtopic, msg); });
    MESSAGE_LIST(CMD_PUB_SUB, RPT_PUB_SUB)
    #undef CMD_PUB_SUB
    #undef RPT_PUB_SUB
  }

private:
  // Convert message to other type and publish
  template <typename In, typename Out>
  void onMessage(std::shared_ptr<rclcpp::Publisher<Out>> pub, const std::shared_ptr<const In> msg) {
    auto out = std::make_unique<Out>();
    ros2_generic_message::Message<In> msg_in(msg.get());
    ros2_generic_message::Message<Out> msg_out(out.get());
    msg_out.set(msg_in);
    
    pub->publish(std::move(out));
  }

  // Publishers and subscribers
  #define DECLARE_PUB_SUB_CMD(xname, xtopic) \
  rclcpp::Subscription<common_ns::xname>::SharedPtr sub_ds_##xtopic; \
  rclcpp::Publisher  <vehicle_ns::xname>::SharedPtr pub_vh_##xtopic;
  #define DECLARE_PUB_SUB_RPT(xname, xtopic) \
  rclcpp::Subscription<vehicle_ns::xname>::SharedPtr sub_vh_##xtopic; \
  rclcpp::Publisher    <common_ns::xname>::SharedPtr pub_ds_##xtopic;
  MESSAGE_LIST(DECLARE_PUB_SUB_CMD, DECLARE_PUB_SUB_RPT)
  #undef DECLARE_PUB_SUB_CMD
  #undef DECLARE_PUB_SUB_RPT
};

} // namespace dataspeed_dbw_gateway

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dataspeed_dbw_gateway::PolarisGateway)
