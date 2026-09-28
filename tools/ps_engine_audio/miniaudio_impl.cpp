// miniaudio_impl.cpp - the ONE translation unit that instantiates miniaudio's
// implementation (MINIAUDIO_IMPLEMENTATION). Every other file in this tool
// includes "miniaudio.h" for declarations only. Keeping the implementation
// in its own TU means the (large, third-party, warnings-laden) generated
// code is compiled exactly once and never pollutes main.cpp's own warning
// baseline.
#if defined(_MSC_VER)
#pragma warning(push, 0)
#endif

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

#if defined(_MSC_VER)
#pragma warning(pop)
#endif
