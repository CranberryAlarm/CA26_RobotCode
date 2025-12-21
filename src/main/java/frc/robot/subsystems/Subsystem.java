package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Subsystem extends SubsystemBase {
  public Subsystem(String baseSmartDashboardKey) {
    this.baseSmartDashboardKey = baseSmartDashboardKey;
  }

  /**
   * Writes the relevant subsystem information to the log
   */
  public void writeToLog() {
  }

  /**
   * Reset sensors, PID Controllers, and any private instance variables
   */
  public abstract void reset();

  /**
   * Contains the algorithms and logic to update the values in mPeriodicIO
   * <p>
   * Do not set the outputs of the system here!!
   * <p>
   * This function is called each periodic cycle before the
   * {@link #writePeriodicOutputs()} function
   */
  public abstract void periodic();

  /**
   * <p>
   * Using the current values in mPeriodicIO, sets the outputs of this subsystem.
   * <p>
   * Examples:
   *
   * <pre>
   *mMotor.set(mPeriodicIO.speed);
   *mSolenoid.set(mPeriodicIO.open);
   *mLED.setRGB(...);
   *etc...
   * </pre>
   * <p>
   * The value of mPeriodicIO variables should not be changed in this function.
   * <p>
   * This function is called each periodic cycle after the {@link #periodic()}
   * function
   */
  public abstract void writePeriodicOutputs();

  /**
   * Stops the subsystem, putting it in a state that is safe for people to touch
   * the robot.
   * <p>
   * Called once when the robot is entering the disabled state
   * {@link #disabledInit()}
   */
  public abstract void stop();

  /**
   * Puts the relevant subsystem information on the SmartDashboard
   */
  public abstract void outputTelemetry();

  public String baseSmartDashboardKey = "UnknownSubsystem/";

  public void putNumber(String key, double value) {
    Logger.recordOutput(baseSmartDashboardKey + "/" + key, value);
  }

  public void putBoolean(String key, boolean value) {
    Logger.recordOutput(baseSmartDashboardKey + "/" + key, value);
  }

  public void putString(String key, String value) {
    Logger.recordOutput(baseSmartDashboardKey + "/" + key, value);
  }

  public void putNumberArray(String key, double[] value) {
    Logger.recordOutput(baseSmartDashboardKey + "/" + key, value);
  }

  public void putBooleanArray(String key, boolean[] value) {
    Logger.recordOutput(baseSmartDashboardKey + "/" + key, value);
  }

  public void putStringArray(String key, String[] value) {
    Logger.recordOutput(baseSmartDashboardKey + "/" + key, value);
  }

  public void putData(String key, Sendable value) {
    SmartDashboard.putData(baseSmartDashboardKey + "/" + key, value);
  }

  public void putNumber(String key, Elevator.ElevatorState value) {
    Logger.recordOutput(baseSmartDashboardKey + "/" + key, value);
  }
}
