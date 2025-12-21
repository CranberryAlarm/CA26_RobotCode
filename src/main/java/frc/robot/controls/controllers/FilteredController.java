package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.controls.Deadband;
import frc.robot.controls.SquaredInput;

public class FilteredController extends GenericHID {
  private static final double DEADBAND_LIMIT = 0.03;

  private boolean m_useDeadband;
  private boolean m_useSquaredInput;

  private Deadband m_deadband = new Deadband(DEADBAND_LIMIT);
  private SquaredInput m_squaredInput = new SquaredInput(DEADBAND_LIMIT);

  private final DPadButton m_up = new DPadButton(this, DPadButton.Direction.UP);
  private final DPadButton m_down = new DPadButton(this, DPadButton.Direction.DOWN);
  private final DPadButton m_left = new DPadButton(this, DPadButton.Direction.LEFT);
  private final DPadButton m_right = new DPadButton(this, DPadButton.Direction.RIGHT);

  public FilteredController(int port) {
    super(port);
    m_useDeadband = false;
    m_useSquaredInput = false;
  }

  public FilteredController(int port, boolean useDeadband, boolean useSquaredInput) {
    this(port);
    this.m_useDeadband = useDeadband;
    this.m_useSquaredInput = useSquaredInput;
  }

  public double getFilteredAxis(int axis) {
    double value = this.getRawAxis(axis);

    // Apply squared input, if requested
    if (m_useSquaredInput) {
      value = m_squaredInput.scale(value);
    }

    // Apply deadband, if requested
    if (m_useDeadband) {
      value = m_deadband.scale(value);
    }

    return value;
  }

  public boolean getHatUp() {
    return m_up.get();
  }

  public boolean getHatDown() {
    return m_down.get();
  }
  
  public boolean getHatLeft() {
    return m_left.get();
  }
  
  public boolean getHatRight() {
    return m_right.get();
  }

  public boolean getHatUpPressed() {
    return m_up.getPressed();
  }

  public boolean getHatDownPressed() {
    return m_down.getPressed();
  }

  public boolean getHatLeftPressed() {
    return m_left.getPressed();
  }

  public boolean getHatRightPressed() {
    return m_right.getPressed();
  }
}
