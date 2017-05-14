/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.ctrlsys;

/**
 * A node for reference inputs (e.g., setpoints).
 */
public class RefInput implements INode {
  private double m_reference;

  public RefInput(double reference) {
    set(reference);
  }

  public RefInput() {
    this(0.0);
  }

  /**
   * Returns value of reference input.
   */
  @Override
  public double getOutput() {
    return m_reference;
  }

  /**
   * Sets reference input.
   */
  public void set(double reference) {
    m_reference = reference;
  }
}
