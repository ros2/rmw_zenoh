#ifndef RMW_ZENOH__RMW_ZENOH_H_
#define RMW_ZENOH__RMW_ZENOH_H_

#include <zenoh.hxx>

#include "rmw/types.h"
#include "rmw/visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

/// Get the Zenoh session associated with the given RMW context.
/// \param[in] context The RMW context.
/// \return A shared pointer to the Zenoh session, or nullptr if invalid.
RMW_PUBLIC
const std::shared_ptr<zenoh::Session>
rmw_zenoh_get_session(const rmw_context_t * context);

#ifdef __cplusplus
}
#endif

#endif  // RMW_ZENOH__RMW_ZENOH_H_