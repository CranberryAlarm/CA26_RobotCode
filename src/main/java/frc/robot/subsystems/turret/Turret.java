package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Turret subsystem using AdvantageKit IO layer pattern.
 * Controls a rotating turret mechanism.
 */
public class Turret extends SubsystemBase {

  // Turret position on robot (from original code)
  public final Translation3d turretTranslation = new Translation3d(-0.205, 0.0, 0.375);

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  /**
   * Creates a new Turret subsystem.
   *
   * @param io the IO implementation to use
   */
  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update and log inputs
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // Log 3D visualization
    Logger.recordOutput("ASCalibration/FinalComponentPoses", new Pose3d[] {
        new Pose3d(
            turretTranslation,
            new Rotation3d(0, 0, Math.toRadians(inputs.positionDegrees)))
    });
  }

  /**
   * Command to set the turret to a specific angle.
   *
   * @param angle the target angle
   * @return a command that sets the turret angle
   */
  public Command setAngle(Angle angle) {
    return Commands.runOnce(() -> io.setPosition(angle.in(Degrees)), this)
        .withName("Turret.SetAngle");
  }

  /**
   * Command to dynamically set the turret angle.
   *
   * @param angleSupplier supplier for the target angle
   * @return a command that continuously updates the turret angle
   */
  public Command setAngleDynamic(Supplier<Angle> angleSupplier) {
    return Commands.run(() -> io.setPosition(angleSupplier.get().in(Degrees)), this)
        .withName("Turret.SetAngleDynamic");
  }

  /**
   * Command to center the turret.
   *
   * @return a command that centers the turret
   */
  public Command center() {
    return setAngle(Degrees.of(0));
  }

  /**
   * Gets the turret angle adjusted for robot coordinate frame.
   * Since the turret is mounted backwards, adds 180 degrees.
   *
   * @return the robot-adjusted angle
   */
  public Angle getRobotAdjustedAngle() {
    return Degrees.of(inputs.positionDegrees + 180);
  }

  /**
   * Gets the raw turret angle.
   *
   * @return the raw encoder angle
   */
  public Angle getRawAngle() {
    return Degrees.of(inputs.positionDegrees);
  }

  /**
   * Command to set duty cycle (open loop).
   *
   * @param dutyCycle the duty cycle [-1, 1]
   * @return a command that sets duty cycle
   */
  public Command set(double dutyCycle) {
    return Commands.runOnce(() -> io.setDutyCycle(dutyCycle), this)
        .withName("Turret.Set");
  }

  /**
   * Command to reset the encoder position to zero.
   *
   * @return a command that resets the encoder
   */
  public Command rezero() {
    return Commands.runOnce(() -> io.resetPosition(), this)
        .withName("Turret.Rezero");
  }

  /**
   * Checks if the turret is at the target position.
   *
   * @return true if at setpoint
   */
  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }
}
