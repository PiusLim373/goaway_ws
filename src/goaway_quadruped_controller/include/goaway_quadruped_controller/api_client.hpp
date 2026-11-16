#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_api/msg/response.hpp>

namespace goaway_quadruped_controller
{

// Client for a Unitree-style SDK service (sport, motion_switcher, goaway_locomotion):
// requests on /api/<service>/request, replies on /api/<service>/response, matched by id.
class ApiClient
{
public:
  struct Reply
  {
    bool answered = false;   // false: no reply before the timeout
    int32_t code = 0;
    std::string data;        // JSON
  };

  ApiClient(rclcpp::Node * node, const std::string & service);

  // Sends and waits for the reply. Blocks only the calling thread: replies arrive in this
  // client's own callback group (needs a multi-threaded executor).
  Reply call(
    int32_t api_id, const std::string & parameter = "",
    std::chrono::milliseconds timeout = std::chrono::milliseconds(3000));

  // Sends without asking for a reply (e.g. sport Move at the cmd_vel rate).
  void send(int32_t api_id, const std::string & parameter = "");

private:
  void on_response(const unitree_api::msg::Response::SharedPtr msg);

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr sub_;

  std::atomic<int64_t> next_id_;
  std::mutex call_mutex_;   // one call at a time
  std::mutex mutex_;
  std::condition_variable cv_;
  int64_t waiting_id_ = 0;
  Reply reply_;
};

}  // namespace goaway_quadruped_controller
