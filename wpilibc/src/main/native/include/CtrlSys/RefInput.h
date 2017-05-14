/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>

namespace frc {

/**
 * A node for reference inputs (e.g., setpoints).
 */
class RefInput {
 public:
  explicit RefInput(double reference = 0.0);
  virtual ~RefInput() = default;

  double GetOutput() const;

  void Set(double reference);

 private:
  double m_reference;
};

}  // namespace frc
