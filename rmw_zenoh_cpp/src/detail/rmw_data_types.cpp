// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include <zenoh.h>

#include <mutex>
#include <utility>

#include "rcutils/logging_macros.h"

#include "rmw_data_types.hpp"

///==============================================================================

saved_msg_data::saved_msg_data(zc_owned_payload_t p, uint64_t recv_ts, const uint8_t pub_gid[16])
: payload(p), recv_timestamp(recv_ts)
{
  memcpy(publisher_gid, pub_gid, 16);
}

void sub_data_handler(
  const z_sample_t * sample,
  void * data)
{
  z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);

  auto sub_data = static_cast<rmw_subscription_data_t *>(data);
  if (sub_data == nullptr) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain rmw_subscription_data_t from data for "
      "subscription for %s",
      z_loan(keystr)
    );
    return;
  }

  {
    std::lock_guard<std::mutex> lock(sub_data->message_queue_mutex);

    if (sub_data->message_queue.size() >= sub_data->queue_depth) {
      // Log warning if message is discarded due to hitting the queue depth
      RCUTILS_LOG_WARN_NAMED(
        "rmw_zenoh_cpp",
        "Message queue depth of %ld reached, discarding oldest message "
        "for subscription for %s",
        sub_data->queue_depth,
        z_loan(keystr));

      std::unique_ptr<saved_msg_data> old = std::move(sub_data->message_queue.back());
      z_drop(&old->payload);
      sub_data->message_queue.pop_back();
    }

    sub_data->message_queue.emplace_front(
      std::make_unique<saved_msg_data>(
        zc_sample_payload_rcinc(sample),
        sample->timestamp.time, sample->timestamp.id.id));

    // Since we added new data, trigger the guard condition if it is available
    std::lock_guard<std::mutex> internal_lock(sub_data->internal_mutex);
    if (sub_data->condition != nullptr) {
      sub_data->condition->notify_one();
    }
  }

  z_drop(z_move(keystr));
}


unsigned int rmw_service_data_t::get_new_uid()
{
  return client_count++;
}

void service_data_handler(const z_query_t * query, void * service_data)
{
  RCUTILS_LOG_INFO_NAMED(
    "rmw_zenoh_cpp",
    "[service_data_handler] triggered"
  );
  z_owned_str_t keystr = z_keyexpr_to_string(z_query_keyexpr(query));

  auto rmw_service_data = static_cast<rmw_service_data_t *>(service_data);
  if (rmw_service_data == nullptr) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain rmw_service_data_t from data for "
      "service for %s",
      z_loan(keystr)
    );
    z_drop(z_move(keystr));
    return;
  }

  // Get the query parameters and payload
  {
    std::lock_guard<std::mutex> lock(rmw_service_data->query_queue_mutex);

    const unsigned int client_id = rmw_service_data->get_new_uid();
    rmw_service_data->id_query_map.emplace(
      std::make_pair(client_id, std::make_unique<z_owned_query_t>(z_query_clone(query))));
    rmw_service_data->to_take.push_back(client_id);


    // Since we added new data, trigger the guard condition if it is available
    std::lock_guard<std::mutex> internal_lock(rmw_service_data->internal_mutex);
    if (rmw_service_data->condition != nullptr) {
      rmw_service_data->condition->notify_one();
    }
  }

  z_drop(z_move(keystr));
}

// void client_data_handler(z_owned_reply_t * reply, void * client_data)
// {
//   auto rmw_client_data = static_cast<rmw_client_data_t *>(client_data);
//   if (rmw_client_data == nullptr) {
//     RCUTILS_LOG_ERROR_NAMED(
//       "rmw_zenoh_cpp",
//       "Unable to obtain rmw_client_data_t "
//     );
//     return;
//   }
//   RCUTILS_LOG_INFO_NAMED(
//     "rmw_zenoh_cpp",
//     "[client_data_handler] triggered for %s",
//     rmw_client_data->service_name
//   );
//   if (!z_check(*reply)) {
//     RCUTILS_LOG_ERROR_NAMED(
//       "rmw_zenoh_cpp",
//       "z_check returned False"
//     );
//     return;
//   }
//   if (!z_reply_check(reply)) {
//     RCUTILS_LOG_ERROR_NAMED(
//       "rmw_zenoh_cpp",
//       "z_reply_check returned False"
//     );
//     return;
//   }
//   if (!z_reply_is_ok(reply)) {
//     RCUTILS_LOG_ERROR_NAMED(
//       "rmw_zenoh_cpp",
//       "z_reply_is_ok returned False"
//     );
//     return;
//   }

//   z_sample_t sample = z_reply_ok(reply);

//   z_owned_str_t keystr = z_keyexpr_to_string(sample.keyexpr);

//   RCUTILS_LOG_DEBUG_NAMED(
//     "rmw_zenoh_cpp",
//     "[client_data_handler] keyexpr of sample: %s",
//     z_loan(keystr)
//   );

//   {
//     std::lock_guard<std::mutex> msg_lock(rmw_client_data->message_mutex);
//     rmw_client_data->message = std::make_unique<saved_msg_data>(
//       zc_sample_payload_rcinc(&sample), sample.timestamp.time, sample.timestamp.id.id);
//   }
//   {
//     std::lock_guard<std::mutex> internal_lock(rmw_client_data->internal_mutex);
//     if (rmw_client_data->condition != nullptr) {
//       rmw_client_data->condition->notify_one();
//     }
//   }

//   z_reply_drop(reply);
//   z_drop(z_move(keystr));
// }
