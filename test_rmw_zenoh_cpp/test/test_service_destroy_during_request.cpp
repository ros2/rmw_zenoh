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

class TestServiceDestroyDuringRequest : public ::testing::Test
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

// Regression test: destroying a service while its zenoh query callback is
// still executing must not deadlock.
//
// The queryable callback captures a weak_ptr to ServiceData and locks it for
// the duration of the callback. If rmw_destroy_service races with an in-flight
// request, the callback's transient shared_ptr can become the last strong
// reference, so ~ServiceData() — and with it the blocking queryable undeclare,
// which drains in-flight callbacks — runs on the very thread executing that
// callback, waiting for itself forever.
//
// rmw_destroy_service is expected to shut the entity down on the calling
// thread instead. That only works if ServiceData::shutdown() actually
// undeclares the queryable, which it skips unless the entity was marked
// initialized.
//
// The test makes the race deterministic: ServiceData::add_new_query()
// synchronously invokes the new-request callback while the query closure still
// holds its strong reference, so blocking inside that callback pins the
// "request in flight" state while the main thread destroys the service.
TEST_F(TestServiceDestroyDuringRequest, DestroyWhileRequestCallbackInFlight)
{
  // Neither node is added to an executor: the request only has to reach the
  // rmw layer, and service destruction order is fully controlled by this test.
  auto service_node = std::make_shared<rclcpp::Node>("destroy_during_request_service_node");
  auto client_node = std::make_shared<rclcpp::Node>("destroy_during_request_client_node");

  auto service = service_node->create_service<test_msgs::srv::BasicTypes>(
    "service_destroy_during_request",
    [](const std::shared_ptr<test_msgs::srv::BasicTypes::Request>,
    std::shared_ptr<test_msgs::srv::BasicTypes::Response> response) {
      response->bool_value = true;
    });

  auto client = client_node->create_client<test_msgs::srv::BasicTypes>(
    "service_destroy_during_request");
  ASSERT_TRUE(poll_until([&]() {return client->service_is_ready();}, 10s));

  std::promise<void> request_in_flight;
  std::promise<void> release_request;
  std::shared_future<void> release_future = release_request.get_future().share();
  std::atomic<bool> first_request{true};

  // Fires inside ServiceData::add_new_query(), on the thread executing the
  // zenoh query closure, while that closure still holds a strong reference
  // to ServiceData.
  service->set_on_new_request_callback(
    [&request_in_flight, release_future, &first_request](size_t) {
      if (first_request.exchange(false)) {
        request_in_flight.set_value();
        release_future.wait_for(10s);
      }
    });

  // In a same-process setup the query is delivered to the local queryable
  // synchronously from rmw_send_request, so the sender thread is the zenoh
  // callback thread for this test.
  std::promise<void> sender_done;
  std::thread sender(
    [&client, &sender_done]() {
      client->async_send_request(std::make_shared<test_msgs::srv::BasicTypes::Request>());
      sender_done.set_value();
    });

  ASSERT_EQ(request_in_flight.get_future().wait_for(10s), std::future_status::ready)
    << "request never reached the service";

  // Keep the request callback blocked while the service is destroyed, then
  // let it finish: pre-fix, the sender thread then drops the last ServiceData
  // reference and self-deadlocks in the queryable undeclare.
  std::thread releaser([&release_request]() {
      std::this_thread::sleep_for(500ms);
      release_request.set_value();
    });

  service.reset();
  releaser.join();

  // The thread that executed the request callback must be able to return.
  const bool alive =
    sender_done.get_future().wait_for(10s) == std::future_status::ready;
  if (!alive) {
    // The sender thread is deadlocked; it can never be joined. Detach so the
    // failure is reported instead of hanging in the thread destructor.
    sender.detach();
    FAIL() << "sender thread never returned after destroying a service during "
      "its request callback: the zenoh callback thread is deadlocked "
      "(~ServiceData ran a blocking undeclare on its own callback thread)";
  }
  sender.join();
}
