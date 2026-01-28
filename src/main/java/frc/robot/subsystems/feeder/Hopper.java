package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Hopper subsystem using AdvantageKit IO layer pattern.
 * Feeds game pieces from intake to kicker/shooter.
 */
public class Hopper extends SubsystemBase {

  private static final double HOPPER_SPEED = 1.0;

  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  /**
   * Creates a new Hopper subsystem.
   *
   * @param io the IO implementation to use
   */
  public Hopper(RollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update and log inputs
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }

  /**
   * Command to run the hopper forward while held.
   *
   * @return a command that feeds forward
   */
  public Command feedCommand() {
    return Commands.startEnd(
        () -> io.setDutyCycle(HOPPER_SPEED),
        () -> io.stop(),
        this).withName("Hopper.Feed");
  }

  /**
   * Command to run the hopper backward while held.
   *
   * @return a command that feeds backward
   */
  public Command backFeedCommand() {
    return Commands.startEnd(
        () -> io.setDutyCycle(-HOPPER_SPEED),
        () -> io.stop(),
        this).withName("Hopper.BackFeed");
  }

  /**
   * Command to run the hopper in reverse while held.
   *
   * @return a command that reverses
   */
  public Command reverseCommand() {
    return backFeedCommand();
  }

  /**
   * Command to stop the hopper.
   *
   * @return a command that stops the hopper
   */
  public Command stopCommand() {
    return Commands.runOnce(() -> io.stop(), this)
        .withName("Hopper.Stop");
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
