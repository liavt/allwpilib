/*----------------------------------------------------------------------------*/
/* Copyright (c) 2011-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>

namespace frc {

class Command;

class CommandGroupEntry {
 public:
  typedef enum {
    kSequence_InSequence,
    kSequence_BranchPeer,
    kSequence_BranchChild
  } Sequence;

  CommandGroupEntry() = default;
  CommandGroupEntry(Command* command, Sequence state, double timeout = -1.0);

  CommandGroupEntry(CommandGroupEntry&&) = default;
  CommandGroupEntry& operator=(CommandGroupEntry&&) = default;

  bool IsTimedOut() const;

  double m_timeout = -1.0;
  std::unique_ptr<Command> m_command;
  Sequence m_state = kSequence_InSequence;
};

}  // namespace frc
