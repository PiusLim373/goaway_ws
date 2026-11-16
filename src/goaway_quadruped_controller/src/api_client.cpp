#include "goaway_quadruped_controller/api_client.hpp"

namespace goaway_quadruped_controller
{

ApiClient::ApiClient(rclcpp::Node * node, const std::string & service)
: next_id_(std::chrono::system_clock::now().time_since_epoch().count())
{
  group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = group_;
  pub_ = node->create_publisher<unitree_api::msg::Request>("/api/" + service + "/request", 10);
  // Best effort matches the service's writer whether it is reliable or not.
  sub_ = node->create_subscription<unitree_api::msg::Response>(
    "/api/" + service + "/response", rclcpp::SensorDataQoS(),
    [this](const unitree_api::msg::Response::SharedPtr msg) {on_response(msg);}, options);
}

ApiClient::Reply ApiClient::call(
  int32_t api_id, const std::string & parameter, std::chrono::milliseconds timeout)
{
  std::lock_guard<std::mutex> one_at_a_time(call_mutex_);
  unitree_api::msg::Request req;
  req.header.identity.id = ++next_id_;
  req.header.identity.api_id = api_id;
  req.parameter = parameter;

  std::unique_lock<std::mutex> lock(mutex_);
  waiting_id_ = req.header.identity.id;
  reply_ = Reply{};
  pub_->publish(req);
  cv_.wait_for(lock, timeout, [this] {return reply_.answered;});
  waiting_id_ = 0;
  return reply_;
}

void ApiClient::send(int32_t api_id, const std::string & parameter)
{
  unitree_api::msg::Request req;
  req.header.identity.id = ++next_id_;
  req.header.identity.api_id = api_id;
  req.header.policy.noreply = true;
  req.parameter = parameter;
  pub_->publish(req);
}

void ApiClient::on_response(const unitree_api::msg::Response::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (waiting_id_ == 0 || msg->header.identity.id != waiting_id_) {return;}
  reply_.answered = true;
  reply_.code = msg->header.status.code;
  reply_.data = msg->data;
  cv_.notify_all();
}

}  // namespace goaway_quadruped_controller
