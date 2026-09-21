// ps/version.h implementation — see that header for scope notes.

#include "ps/version.h"

#define PS_VERSION_MAJOR 0
#define PS_VERSION_MINOR 1
#define PS_VERSION_PATCH 0

#define PS_STRINGIFY_IMPL(x) #x
#define PS_STRINGIFY(x) PS_STRINGIFY_IMPL(x)

namespace ps {

const char* version_string() {
    return PS_STRINGIFY(PS_VERSION_MAJOR) "." PS_STRINGIFY(PS_VERSION_MINOR) "." PS_STRINGIFY(PS_VERSION_PATCH);
}

} // namespace ps
