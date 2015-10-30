/*----------------------------------------------------------------------------*/
/* Copyright (c) 2014-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "command/MockCommand.h"

using namespace frc;

MockCommand::MockCommand(Subsystem* subsys) {
  Requires(subsys);
}

int32_t MockCommand::GetInitializeCount() const { return m_initializeCount; }

bool MockCommand::HasInitialized() const { return GetInitializeCount() > 0; }

int32_t MockCommand::GetExecuteCount() const { return m_executeCount; }

int32_t MockCommand::GetIsFinishedCount() const { return m_isFinishedCount; }

bool MockCommand::IsHasFinished() const { return m_hasFinished; }

void MockCommand::SetHasFinished(bool hasFinished) {
  m_hasFinished = hasFinished;
}

int32_t MockCommand::GetEndCount() const { return m_endCount; }

bool MockCommand::HasEnd() const { return GetEndCount() > 0; }

int32_t MockCommand::GetInterruptedCount() const { return m_interruptedCount; }

bool MockCommand::HasInterrupted() const { return GetInterruptedCount() > 0; }

void MockCommand::Initialize() { ++m_initializeCount; }

void MockCommand::Execute() { ++m_executeCount; }

bool MockCommand::IsFinished() {
  ++m_isFinishedCount;
  return IsHasFinished();
}

void MockCommand::End() { ++m_endCount; }

void MockCommand::Interrupted() { ++m_interruptedCount; }

void MockCommand::ResetCounters() {
  m_initializeCount = 0;
  m_executeCount = 0;
  m_isFinishedCount = 0;
  m_hasFinished = false;
  m_endCount = 0;
  m_interruptedCount = 0;
}
