package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;

public class OperatorControls {

  public static void configure(int port, SwerveSubsystem drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    // Intake controls - A to intake, B to eject
    controller.a().whileTrue(superstructure.intakeCommand());
    controller.b().whileTrue(superstructure.ejectCommand());

    // Hopper controls - X to run hopper forward, Y to run backward
    controller.x().whileTrue(superstructure.feedCommand());
    controller.y().whileTrue(superstructure.hopperReverseCommand());
  }
}
