// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <test_msgs/srv/basic_types.hpp>

using namespace std::chrono_literals;

namespace
{
template<typename Pred>
bool poll_until(Pred && pred, std::chrono::seconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return pred();
}
}  // namespace

class TestClientDestroyDuringReply : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

// Regression test: destroying a client while its zenoh reply callback is
// still executing must not deadlock.
//
// The reply callback captures a weak_ptr to ClientData and locks it for the
// duration of the callback. If rmw_destroy_client races with an in-flight
// reply, the callback's transient shared_ptr can become the last strong
// reference, so ~ClientData() — and with it the blocking querier undeclare,
// which drains in-flight callbacks — runs on the very thread executing that
// callback, waiting for itself forever.
//
// The test makes the race deterministic: ClientData::add_new_reply()
// synchronously invokes the new-response callback while the reply closure
// still holds its strong reference, so blocking inside that callback pins
// the "reply in flight" state while the main thread destroys the client.
TEST_F(TestClientDestroyDuringReply, DestroyWhileReplyCallbackInFlight)
{
  // The service lives on its own node, spun by a dedicated executor thread.
  // The client node is intentionally never added to an executor so that
  // client destruction order is fully controlled by this test.
  auto service_node = std::make_shared<rclcpp::Node>("destroy_during_reply_service_node");
  auto client_node = std::make_shared<rclcpp::Node>("destroy_during_reply_client_node");

  auto service = service_node->create_service<test_msgs::srv::BasicTypes>(
    "client_destroy_during_reply",
    [](const std::shared_ptr<test_msgs::srv::BasicTypes::Request>,
    std::shared_ptr<test_msgs::srv::BasicTypes::Response> response) {
      response->bool_value = true;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(service_node);
  std::thread spinner([&executor]() {executor.spin();});

  auto client = client_node->create_client<test_msgs::srv::BasicTypes>(
    "client_destroy_during_reply");
  ASSERT_TRUE(poll_until([&]() {return client->service_is_ready();}, 10s));

  std::promise<void> reply_in_flight;
  std::promise<void> release_reply;
  std::shared_future<void> release_future = release_reply.get_future().share();
  std::atomic<bool> first_reply{true};

  // Fires inside ClientData::add_new_reply(), on the thread executing the
  // zenoh reply closure, while that closure still holds a strong reference
  // to ClientData.
  client->set_on_new_response_callback(
    [&reply_in_flight, release_future, &first_reply](size_t) {
      if (first_reply.exchange(false)) {
        reply_in_flight.set_value();
        release_future.wait_for(10s);
      }
    });

  client->async_send_request(std::make_shared<test_msgs::srv::BasicTypes::Request>());

  ASSERT_EQ(reply_in_flight.get_future().wait_for(10s), std::future_status::ready)
    << "service reply never reached the client";

  // Keep the reply callback blocked while the client is destroyed, then let
  // it finish: pre-fix, the callback thread then drops the last ClientData
  // reference and self-deadlocks in the querier undeclare.
  std::thread releaser([&release_reply]() {
      std::this_thread::sleep_for(500ms);
      release_reply.set_value();
    });

  client.reset();
  releaser.join();

  // Probe: the thread that executed the reply callback (the service node's
  // spinner, since replies are delivered inline from rmw_send_response in a
  // same-process setup) must still be alive and able to serve a new call.
  auto probe = client_node->create_client<test_msgs::srv::BasicTypes>(
    "client_destroy_during_reply");
  ASSERT_TRUE(poll_until([&]() {return probe->service_is_ready();}, 10s));

  std::promise<void> probe_replied;
  std::atomic<bool> probe_first{true};
  probe->set_on_new_response_callback(
    [&probe_replied, &probe_first](size_t) {
      if (probe_first.exchange(false)) {
        probe_replied.set_value();
      }
    });
  probe->async_send_request(std::make_shared<test_msgs::srv::BasicTypes::Request>());

  const bool alive =
    probe_replied.get_future().wait_for(10s) == std::future_status::ready;
  if (!alive) {
    // The executor thread is deadlocked; it can never be joined. Detach so
    // the failure is reported instead of hanging in the thread destructor.
    spinner.detach();
    FAIL() << "no reply after destroying a client during its reply callback: "
      "the zenoh callback thread is deadlocked (~ClientData ran a "
      "blocking undeclare on its own callback thread)";
  }

  executor.cancel();
  spinner.join();
}
