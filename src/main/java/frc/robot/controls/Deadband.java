package frc.robot.controls;

public class Deadband implements InputScaler {
  private final double m_deadbandLimit;

  public Deadband(double deadbandLimit) {
    this.m_deadbandLimit = deadbandLimit;
  }

  public double scale(double input) {
    return Math.abs(input) > m_deadbandLimit ? input : 0;
  }
}
