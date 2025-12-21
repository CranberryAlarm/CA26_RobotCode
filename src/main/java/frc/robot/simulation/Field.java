package frc.robot.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Field extends Field2d {
  private static Field m_field;

  public static Field getInstance() {
    if (m_field == null) {
      m_field = new Field();
    }
    return m_field;
  }

  private Field() {
    super();
  }
}
