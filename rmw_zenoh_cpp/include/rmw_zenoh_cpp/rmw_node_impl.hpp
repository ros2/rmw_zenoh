#ifndef RMW_ZENOH_CPP__RMW_NODE_IMPL_HPP_
#define RMW_ZENOH_CPP__RMW_NODE_IMPL_HPP_

#include "rmw/rmw.h"

extern "C"
{
  #include "zenoh/zenoh-ffi.h"
}

struct rmw_node_impl_t
{
  rmw_guard_condition_t * graph_guard_condition_;
};

#endif // RMW_ZENOH_CPP__RMW_NODE_IMPL_HPP_
