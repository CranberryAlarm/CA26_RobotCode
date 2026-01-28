package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Kicker subsystem using AdvantageKit IO layer pattern.
 * Final roller that kicks game pieces into the shooter.
 */
public class Kicker extends SubsystemBase {

  private static final double KICKER_SPEED = 1.0;

  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  /**
   * Creates a new Kicker subsystem.
   *
   * @param io the IO implementation to use
   */
  public Kicker(RollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update and log inputs
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  /**
   * Command to run the kicker forward while held.
   *
   * @return a command that feeds forward
   */
  public Command feedCommand() {
    return Commands.startEnd(
        () -> io.setDutyCycle(KICKER_SPEED),
        () -> io.stop(),
        this).withName("Kicker.Feed");
  }

  /**
   * Command to run the kicker backward while held.
   *
   * @return a command that reverses
   */
  public Command reverseCommand() {
    return Commands.startEnd(
        () -> io.setDutyCycle(-KICKER_SPEED),
        () -> io.stop(),
        this).withName("Kicker.Reverse");
  }

  /**
   * Command to stop the kicker.
   *
   * @return a command that stops the kicker
   */
  public Command stopCommand() {
    return Commands.runOnce(() -> io.stop(), this)
        .withName("Kicker.Stop");
  }

  /**
   * Gets the current roller velocity in RPM.
   *
   * @return the velocity in RPM
   */
  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }
}
