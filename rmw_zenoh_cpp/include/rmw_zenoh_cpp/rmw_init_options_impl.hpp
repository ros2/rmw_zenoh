#ifndef RMW_ZENOH_CPP__RMW_INIT_OPTIONS_IMPL_HPP_
#define RMW_ZENOH_CPP__RMW_INIT_OPTIONS_IMPL_HPP_

struct rmw_init_options_impl_t
{
  char * session_locator;  // Zenoh session TCP locator
  char * mode;  // Zenoh session mode
};

#endif  // RMW_ZENOH_CPP__RMW_INIT_OPTIONS_IMPL_HPP_
