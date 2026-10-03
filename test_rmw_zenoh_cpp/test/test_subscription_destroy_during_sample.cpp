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
#include <test_msgs/msg/empty.hpp>

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

class TestSubscriptionDestroyDuringSample : public ::testing::Test
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

// Regression test: destroying a subscription while zenoh sample callbacks are
// in flight must not deadlock.
//
// The sample callback locks SubscriptionData::mutex_ in add_new_message().
// SubscriptionData::shutdown() must therefore release mutex_ before it
// undeclares the subscriber: the undeclare blocks until every in-flight sample
// callback has returned, and a callback that is waiting for mutex_ never
// returns while shutdown() holds it (AB-BA between the application thread and
// the callback thread).
//
// The test makes the race deterministic. In a same-process setup a publication
// is delivered to the local subscriber synchronously on the publishing thread,
// so two publisher threads play the role of two zenoh callback threads:
//   1. thread A publishes and is parked inside the new-message callback, which
//      add_new_message() invokes while holding mutex_;
//   2. the destroyer thread enters shutdown() and queues up on mutex_;
//   3. thread B publishes and queues up on mutex_ in add_new_message();
//   4. thread A is released. The destroyer takes mutex_ and undeclares the
//      subscriber while thread B is still in flight.
// Pre-fix, step 4 deadlocks: the undeclare waits for B, and B waits for the
// mutex_ the destroyer is holding across the undeclare.
TEST_F(TestSubscriptionDestroyDuringSample, DestroyWhileSampleCallbacksInFlight)
{
  // Nodes are intentionally never added to an executor so that subscription
  // destruction order is fully controlled by this test.
  auto pub_node = std::make_shared<rclcpp::Node>("destroy_during_sample_pub_node");
  auto sub_node = std::make_shared<rclcpp::Node>("destroy_during_sample_sub_node");

  // Two publishers: PublisherData::publish() serializes on its own mutex, so
  // two concurrent publications need two publishers.
  auto pub_a = pub_node->create_publisher<test_msgs::msg::Empty>(
    "subscription_destroy_during_sample", 10);
  auto pub_b = pub_node->create_publisher<test_msgs::msg::Empty>(
    "subscription_destroy_during_sample", 10);

  auto sub = sub_node->create_subscription<test_msgs::msg::Empty>(
    "subscription_destroy_during_sample", 10,
    [](test_msgs::msg::Empty::ConstSharedPtr) {});
  ASSERT_TRUE(poll_until([&]() {return pub_a->get_subscription_count() >= 1;}, 10s));
  ASSERT_TRUE(poll_until([&]() {return pub_b->get_subscription_count() >= 1;}, 10s));

  std::promise<void> sample_in_flight;
  std::promise<void> release_sample;
  std::shared_future<void> release_future = release_sample.get_future().share();
  std::atomic<bool> first_sample{true};

  // Fires inside SubscriptionData::add_new_message(), on the thread executing
  // the zenoh sample closure, with SubscriptionData::mutex_ held and the
  // closure still holding a strong reference to SubscriptionData.
  sub->set_on_new_message_callback(
    [&sample_in_flight, release_future, &first_sample](size_t) {
      if (first_sample.exchange(false)) {
        sample_in_flight.set_value();
        release_future.wait_for(10s);
      }
    });

  // Step 1: park thread A inside the callback.
  std::promise<void> publisher_a_done;
  std::thread publisher_a(
    [&pub_a, &publisher_a_done]() {
      pub_a->publish(test_msgs::msg::Empty());
      publisher_a_done.set_value();
    });
  ASSERT_EQ(sample_in_flight.get_future().wait_for(10s), std::future_status::ready)
    << "sample never reached the subscription";

  // Step 2: destroy the subscription. shutdown() blocks on mutex_ behind A.
  std::promise<void> destroy_done;
  std::thread destroyer(
    [&sub, &destroy_done]() {
      sub.reset();
      destroy_done.set_value();
    });
  std::this_thread::sleep_for(200ms);

  // Step 3: a second in-flight sample, blocked on mutex_ in add_new_message().
  std::promise<void> publisher_b_done;
  std::thread publisher_b(
    [&pub_b, &publisher_b_done]() {
      pub_b->publish(test_msgs::msg::Empty());
      publisher_b_done.set_value();
    });
  std::this_thread::sleep_for(200ms);

  // Step 4: let A go. The destroyer now undeclares while B is in flight.
  release_sample.set_value();

  const bool destroyed =
    destroy_done.get_future().wait_for(10s) == std::future_status::ready;
  const bool b_returned =
    publisher_b_done.get_future().wait_for(10s) == std::future_status::ready;
  const bool a_returned =
    publisher_a_done.get_future().wait_for(10s) == std::future_status::ready;
  if (!destroyed || !b_returned || !a_returned) {
    // Deadlocked threads can never be joined. Detach so the failure is
    // reported instead of hanging in the thread destructors.
    destroyer.detach();
    publisher_b.detach();
    publisher_a.detach();
    FAIL() << "deadlock destroying a subscription with sample callbacks in flight "
      "(destroy returned: " << destroyed << ", publisher A returned: " << a_returned <<
      ", publisher B returned: " << b_returned << "): SubscriptionData::shutdown() "
      "undeclared the subscriber while holding mutex_";
  }
  destroyer.join();
  publisher_b.join();
  publisher_a.join();
}
