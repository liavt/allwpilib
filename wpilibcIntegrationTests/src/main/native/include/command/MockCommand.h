/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Commands/Command.h"

namespace frc {

class MockCommand : public Command {
 public:
  MockCommand() = default;
  explicit MockCommand(Subsystem*);

  int32_t GetInitializeCount() const;
  bool HasInitialized() const;

  int32_t GetExecuteCount() const;
  int32_t GetIsFinishedCount() const;
  bool IsHasFinished() const;
  void SetHasFinished(bool hasFinished);
  int32_t GetEndCount() const;
  bool HasEnd() const;

  int32_t GetInterruptedCount() const;
  bool HasInterrupted() const;
  void ResetCounters();

 protected:
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;

 private:
  int32_t m_initializeCount = 0;
  int32_t m_executeCount = 0;
  int32_t m_isFinishedCount = 0;
  bool m_hasFinished = false;
  int32_t m_endCount = 0;
  int32_t m_interruptedCount = 0;
};

}  // namespace frc
