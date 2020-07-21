#ifndef RMW_ZENOH_CPP__RMW_INIT_OPTIONS_IMPL_HPP_
#define RMW_ZENOH_CPP__RMW_INIT_OPTIONS_IMPL_HPP_

struct rmw_init_options_impl_t
{
  const char * session_locator;  // Zenoh session TCP locator
  bool client_mode;  // Init Zenoh session in client mode
};

#endif // RMW_ZENOH_CPP__RMW_INIT_OPTIONS_IMPL_HPP_
