#ifndef RMW_ZENOH_CPP__RMW_CONTEXT_IMPL_HPP_
#define RMW_ZENOH_CPP__RMW_CONTEXT_IMPL_HPP_

extern "C"
{
  #include "zenoh/zenoh-ffi.h"
}

struct rmw_context_impl_t
{
  ZNSession * session;
};

#endif // RMW_ZENOH_CPP__RMW_CONTEXT_IMPL_HPP_
