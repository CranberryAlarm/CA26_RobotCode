package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(AlgaeSubsystem subsystem) {
    return Commands.sequence(new ExampleCommand(subsystem));
    // return Commands.sequence(subsystem.exampleMethodCommand(), new
    // ExampleCommand(subsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
