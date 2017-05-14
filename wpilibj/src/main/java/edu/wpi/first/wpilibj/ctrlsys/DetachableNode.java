/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.ctrlsys;

/**
 * Provides a common base class for nodes with one input.
 *
 * <p>This allows changing the input (i.e., graph structure) from one place.
 */
public abstract class DetachableNode implements INode {
  private INode m_input;

  public DetachableNode(INode input) {
    setInput(input);
  }

  /**
   * Change input node.
   *
   * <p>Warning: This operation is not thread-safe. Only call this when the Output instance using a
   * graph containing this node is disabled.
   */
  public void setInput(INode input) {
    m_input = input;
  }

  /**
   * Get input node.
   *
   * <p>Warning: This operation is not thread-safe. Only call this when the Output instance using a
   * graph containing this node is disabled.
   */
  public INode getInput() {
    return m_input;
  }

  @Override
  public double getOutput() {
    return m_input.getOutput();
  }
}
