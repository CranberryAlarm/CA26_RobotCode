package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the shooter subsystem.
 * Abstracts hardware interactions for logging and replay support.
 */
public interface ShooterIO {

  /**
   * Input data for the shooter subsystem.
   * All sensor readings and motor feedback should be captured here.
   */
  @AutoLog
  public static class ShooterIOInputs {
    /** Current velocity of the leader motor in RPM */
    public double leaderVelocityRPM = 0.0;

    /** Current velocity of the follower motor in RPM */
    public double followerVelocityRPM = 0.0;

    /** Current draw of the leader motor in amps */
    public double leaderCurrentAmps = 0.0;

    /** Current draw of the follower motor in amps */
    public double followerCurrentAmps = 0.0;

    /** Applied voltage to the leader motor */
    public double leaderAppliedVolts = 0.0;

    /** Temperature of the leader motor in Celsius */
    public double leaderTempCelsius = 0.0;

    /** Temperature of the follower motor in Celsius */
    public double followerTempCelsius = 0.0;

    /** Current setpoint velocity in RPM (0 if open loop) */
    public double velocitySetpointRPM = 0.0;

    /** Whether the shooter is at the target speed */
    public boolean atSetpoint = false;
  }

  /**
   * Updates all input values. Should be called once per cycle.
   *
   * @param inputs the inputs object to update
   */
  default void updateInputs(ShooterIOInputs inputs) {
  }

  /**
   * Sets the shooter to a target velocity using closed-loop control.
   *
   * @param velocityRPM the target velocity in RPM
   */
  default void setVelocity(double velocityRPM) {
  }

  /**
   * Sets the shooter motors to a duty cycle (open loop).
   *
   * @param dutyCycle the duty cycle [-1, 1]
   */
  default void setDutyCycle(double dutyCycle) {
  }

  /**
   * Stops the shooter motors.
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
