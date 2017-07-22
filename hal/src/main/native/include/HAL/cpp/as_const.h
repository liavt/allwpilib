/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

// Define as_const for C++11-only compilers
#if __cplusplus < 201703L
#include <type_traits>

namespace std {
#if __cpluplus < 201402L
template <class T>
using add_const_t = typename add_const<T>::type;
#endif

template <class T>
constexpr std::add_const_t<T>& as_const(T& t) noexcept {
  return t;
}

template <class T>
void as_const(const T&&) = delete;
}  // namespace std
#endif
