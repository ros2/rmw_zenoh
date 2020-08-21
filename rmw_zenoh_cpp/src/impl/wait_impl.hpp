#ifndef RMW_ZENOH_CPP_TYPES__WAIT_SET_DATA_HPP_
#define RMW_ZENOH_CPP_TYPES__WAIT_SET_DATA_HPP_

#include <condition_variable>
#include <mutex>

#include "rmw/rmw.h"
#include "rmw/event.h"

#include "guard_condition_impl.hpp"

typedef struct rmw_wait_set_data_t
{
  std::condition_variable condition;
  std::mutex condition_mutex;
} rmw_wait_set_data_t;

/// HELPER FUNCTION FOR WAIT ===================================================
bool check_wait_conditions(
  const rmw_subscriptions_t * subscriptions,
  const rmw_guard_conditions_t * guard_conditions,
  const rmw_services_t * services,
  const rmw_clients_t * clients,
  const rmw_events_t * events
);

#endif  // RMW_ZENOH_CPP_TYPES__WAIT_SET_DATA_HPP_
