/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <tuple>

#include "Base.h"
#include "CtrlSys/Output.h"
#include "Notifier.h"

namespace frc {

/**
 * Allows grouping Output instances together to run in one thread.
 *
 * Each output's OutputFunc() is called at a regular interval. This can be used
 * to avoid unnecessary context switches for Output instances that are running
 * at the same sample rate and priority.
 */
template <typename Out, typename... Outs>
class OutputGroup {
 public:
  explicit OutputGroup(Out& output, Outs&&... outputs);

  virtual ~OutputGroup() = default;

  void Enable(double period = kDefaultPeriod);
  void Disable();

 protected:
  virtual void OutputFunc();

 private:
  std::tuple<Out, Outs...> m_outputs;
  Notifier m_thread;
};

}  // namespace frc

#include "CtrlSys/OutputGroup.inc"
