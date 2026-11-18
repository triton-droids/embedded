/** @file cv_bridge_compat.hpp
 * @brief Select the cv_bridge public header available in the active ROS distro.
 * This handles the header-name difference only; it does not resolve OpenCV ABI
 * mismatches between the installed cv_bridge and a separately selected SDK.
 */
#pragma once

// Jazzy provides .hpp; Humble provides .h.
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
