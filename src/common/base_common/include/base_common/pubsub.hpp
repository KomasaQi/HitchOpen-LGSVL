// Copyright 2021 Gaia Platform LLC

#ifndef BASE_COMMON__PUBSUB_HPP_
#define BASE_COMMON__PUBSUB_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace pubsub
{

template<typename MessageT, typename NodeT = rclcpp::Node>
class MsgSubscriber
{
public:
  // cppcheck-suppress unknownMacro
  RCLCPP_SMART_PTR_DEFINITIONS(MsgSubscriber<MessageT, NodeT>)

  MsgSubscriber(
    NodeT * const parent,
    std::string const & topic_name,
    rclcpp::QoS const & qos)
  {
    parent_ = parent;
    latest_msg_time_ = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);

    subscription_ =
      parent->template create_subscription<MessageT>(
      topic_name, qos,
      [this](const typename MessageT::SharedPtr msg) {on_msg_received(msg);});
  }

  typename MessageT::SharedPtr take()
  {
    auto msg = last_received_msg_;
    last_received_msg_ = nullptr;
    return msg;
  }

  bool has_seen_msg()
  {
    return has_seen_msg_;
  }

  bool has_msg()
  {
    return last_received_msg_ != nullptr;
  }

  rclcpp::Time latest_msg_time()
  {
    return latest_msg_time_;
  }

  [[nodiscard]] typename MessageT::SharedPtr last_received_msg() const
  {
    return last_received_msg_;
  }

private:
  bool has_seen_msg_{};
  rclcpp::Time latest_msg_time_;
  NodeT * parent_;

  typename MessageT::SharedPtr last_received_msg_;
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;

  void on_msg_received(const typename MessageT::SharedPtr msg)
  {
    has_seen_msg_ = true;
    last_received_msg_ = msg;
    latest_msg_time_ = parent_->now();
  }
};

template<class MsgT>
void publish_to(
  rclcpp::Node * this_ptr,
  typename std::shared_ptr<rclcpp::Publisher<MsgT>> & publisher,
  const std::string & topic_name,
  const rclcpp::QoS & qos = rclcpp::SensorDataQoS())
{
  publisher = this_ptr->create_publisher<MsgT>(topic_name, qos);
}

template<class MsgT>
void publish_to(
  rclcpp_lifecycle::LifecycleNode * this_ptr,
  typename std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MsgT>> & publisher,
  const std::string & topic_name,
  const rclcpp::QoS & qos = rclcpp::SensorDataQoS())
{
  publisher = this_ptr->create_publisher<MsgT>(topic_name, qos);
}

template<class MsgT>
void activate_publisher(
  typename std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MsgT>> & publisher)
{
  publisher->on_activate();
}

template<class MsgT>
void deactivate_publisher(
  typename std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MsgT>> & publisher)
{
  publisher->on_deactivate();
}

template<class MsgT, class NodeT>
void subscribe_from(
  NodeT * this_ptr,
  typename std::shared_ptr<rclcpp::Subscription<MsgT>> & subscriber,
  const std::string & topic_name,
  void (NodeT::* callback)(typename std::shared_ptr<MsgT>),
  const rclcpp::QoS & qos = rclcpp::SensorDataQoS())
{
  subscriber =
    static_cast<rclcpp::Node *>(this_ptr)->create_subscription<MsgT>(
    topic_name, qos,
    std::bind(callback, this_ptr, std::placeholders::_1));
}

template<class MsgT>
void subscribe_from(
  rclcpp::Node * const parent,
  typename std::unique_ptr<MsgSubscriber<MsgT>> & subscriber,
  const std::string & topic_name,
  const rclcpp::QoS & qos = rclcpp::SensorDataQoS())
{
  subscriber = std::make_unique<MsgSubscriber<MsgT>>(parent, topic_name, qos);
}

template<class MsgT>
void subscribe_from(
  rclcpp_lifecycle::LifecycleNode * const parent,
  typename std::unique_ptr<MsgSubscriber<MsgT, rclcpp_lifecycle::LifecycleNode>> & subscriber,
  const std::string & topic_name,
  const rclcpp::QoS & qos = rclcpp::SensorDataQoS())
{
  subscriber = std::make_unique<MsgSubscriber<MsgT, rclcpp_lifecycle::LifecycleNode>>(
    parent,
    topic_name,
    qos);
}

template<class MsgT>
void subscribe_from(
  rclcpp::Node * const parent,
  typename std::shared_ptr<MsgSubscriber<MsgT>> & subscriber,
  const std::string & topic_name,
  const rclcpp::QoS & qos = rclcpp::SensorDataQoS())
{
  subscriber = std::make_shared<MsgSubscriber<MsgT>>(parent, topic_name, qos);
}

}  // namespace pubsub

#endif  // BASE_COMMON__PUBSUB_HPP_
