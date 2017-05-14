/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.ctrlsys;

import edu.wpi.first.wpilibj.Notifier;

/**
 * Allows grouping Output instances together to run in one thread.
 *
 * <p>Each output's OutputFunc() is called at a regular interval. This can be used to avoid
 * unnecessary context switches for Output instances that are running at the same sample rate and
 * priority.
 */
public class OutputGroup {
  private Output[] m_outputs;
  private Notifier m_thread = new Notifier(() -> {
    outputFunc();
  });

  /**
   * Appends output to array.
   *
   * @param outputs the Output objects to add to the array for round robin
   */
  public OutputGroup(Output... outputs) {
    m_outputs = outputs;
  }

  /**
   * Starts closed loop control.
   *
   * @param period the loop time for doing calculations. The default is 50ms.
   */
  public void enable(double period) {
    m_thread.startPeriodic(period);
  }

  /**
   * Starts closed loop control.
   */
  public void enable() {
    enable(INode.DEFAULT_PERIOD);
  }

  /**
   * Stops closed loop control.
   */
  public void disable() {
    m_thread.stop();
  }

  protected void outputFunc() {
    for (Output output : m_outputs) {
      output.outputFunc();
    }
  }
}
