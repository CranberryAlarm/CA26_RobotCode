package frc.robot.controls;

public class SquaredInput implements InputScaler {
  private final double m_deadbandLimit;

  public SquaredInput(double deadbandLimit) {
    this.m_deadbandLimit = deadbandLimit;
  }

  public double scale(double input) {
    double numeratorFactor = (Math.abs(input) - m_deadbandLimit);
    double denominatorFactor = 1 - m_deadbandLimit;
    double sign = (input >= 0 ? 1 : -1);

    return sign * ((numeratorFactor * numeratorFactor) / (denominatorFactor * denominatorFactor));
  }
}
