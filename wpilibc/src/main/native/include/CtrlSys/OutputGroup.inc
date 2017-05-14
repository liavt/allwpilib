/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <utility>

namespace frc {

/**
 * Appends output to array.
 *
 * @param output the Output object to add to the array for round robin
 * @param outputs the other Output objects
 */
template <typename Out, typename... Outs>
OutputGroup<Out, Outs...>::OutputGroup(Out& output, Outs&&... outputs)
    : m_outputs{output, outputs...}, m_thread(&OutputGroup::OutputFunc, this) {}

/**
 * Starts closed loop control.
 *
 * @param period the loop time for doing calculations.
 */
template <typename Out, typename... Outs>
void OutputGroup<Out, Outs...>::Enable(double period) {
  m_thread.StartPeriodic(period);
}

/**
 * Stops closed loop control.
 */
template <typename Out, typename... Outs>
void OutputGroup<Out, Outs...>::Disable() {
  m_thread.Stop();
}

namespace detail {

template <typename Out, typename... Us>
void Call(Out&& output, Us&&... us) {
  output.OutputFunc();
  Call(std::forward<Us>(us)...);
}

template <typename Out>
void Call(Out&& output) {
  output.OutputFunc();
}

template <typename Tuple, size_t... Is>
void CallImpl(const Tuple& t, std::index_sequence<Is...>) {
  Call(std::get<Is>(t)...);
}

}  // namespace detail

template <typename Output, typename... Outputs>
void OutputGroup<Output, Outputs...>::OutputFunc() {
  detail::CallImpl(m_outputs, std::index_sequence_for<Output, Outputs...>());
}

}  // namespace frc
