/** @file target_filter.hpp
 * @brief Reserved target-state filter interface; no estimator is implemented here.
 * Current visual tracking holds the latest error and limits command dynamics;
 * it does not perform Kalman filtering or infer target velocity.
 */
#pragma once
