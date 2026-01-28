package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Hood subsystem using AdvantageKit IO layer pattern.
 * Controls the shooter hood angle for trajectory adjustment.
 */
public class Hood extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  /**
   * Creates a new Hood subsystem.
   *
   * @param io the IO implementation to use
   */
  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update and log inputs
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  /**
   * Command to set the hood to a specific angle.
   *
   * @param angle the target angle
   * @return a command that sets the hood angle
   */
  public Command setAngle(Angle angle) {
    return Commands.runOnce(() -> io.setPosition(angle.in(Degrees)), this)
        .withName("Hood.SetAngle");
  }

  /**
   * Command to dynamically set the hood angle.
   *
   * @param angleSupplier supplier for the target angle
   * @return a command that continuously updates the hood angle
   */
  public Command setAngleDynamic(Supplier<Angle> angleSupplier) {
    return Commands.run(() -> io.setPosition(angleSupplier.get().in(Degrees)), this)
        .withName("Hood.SetAngleDynamic");
  }

  /**
   * Command to stow the hood (angle = 0).
   *
   * @return a command that stows the hood
   */
  public Command stow() {
    return setAngle(Degrees.of(0));
  }

  /**
   * Command to set hood to maximum angle.
   *
   * @return a command that sets max angle
   */
  public Command max() {
    return setAngle(Degrees.of(90));
  }

  /**
   * Gets the current hood angle.
   *
   * @return the current angle
   */
  public Angle getAngle() {
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
        .withName("Hood.Set");
  }

  /**
   * Checks if the hood is at the target position.
   *
   * @return true if at setpoint
   */
  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }
}
