/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "CtrlSys/RefInput.h"

using namespace frc;

RefInput::RefInput(double reference) { Set(reference); }

/**
 * Returns value of reference input.
 */
double RefInput::GetOutput() const { return m_reference; }

/**
 * Sets reference input.
 */
void RefInput::Set(double reference) { m_reference = reference; }
