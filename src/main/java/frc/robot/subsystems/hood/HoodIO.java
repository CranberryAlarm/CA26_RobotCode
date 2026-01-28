package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the hood subsystem.
 * Abstracts hardware interactions for logging and replay support.
 */
public interface HoodIO {

  /**
   * Input data for the hood subsystem.
   */
  @AutoLog
  public static class HoodIOInputs {
    /** Current hood angle in degrees */
    public double positionDegrees = 0.0;

    /** Current hood velocity in degrees per second */
    public double velocityDegreesPerSec = 0.0;

    /** Current draw in amps */
    public double currentAmps = 0.0;

    /** Applied voltage */
    public double appliedVolts = 0.0;

    /** Motor temperature in Celsius */
    public double tempCelsius = 0.0;

    /** Target position setpoint in degrees */
    public double setpointDegrees = 0.0;

    /** Whether the hood is at the target position */
    public boolean atSetpoint = false;
  }

  /**
   * Updates all input values. Should be called once per cycle.
   *
   * @param inputs the inputs object to update
   */
  default void updateInputs(HoodIOInputs inputs) {
  }

  /**
   * Sets the hood to a target angle using closed-loop control.
   *
   * @param angleDegrees the target angle in degrees
   */
  default void setPosition(double angleDegrees) {
  }

  /**
   * Sets the hood motor to a duty cycle (open loop).
   *
   * @param dutyCycle the duty cycle [-1, 1]
   */
  default void setDutyCycle(double dutyCycle) {
  }

  /**
   * Stops the hood motor.
   */
  default void stop() {
  }

  /**
   * Sets voltage directly (for SysId).
   *
   * @param volts the voltage to apply
   */
  default void setVoltage(double volts) {
  }

  /**
   * Configures the closed-loop PID gains.
   *
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   */
  default void configurePID(double kP, double kI, double kD) {
  }
}
