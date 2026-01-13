package frc.robot.controls;

import static edu.wpi.first.units.Units.Degrees;

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
    controller.x().whileTrue(superstructure.hopperFeedCommand());
    controller.y().whileTrue(superstructure.hopperReverseCommand());

    // Shooter controls - Right bumper to shoot
    controller.rightBumper().whileTrue(superstructure.shootCommand());
    controller.leftBumper().whileTrue(superstructure.stopShootingCommand());

    // Intake pivot controls. Setpoints need to be tested and finalized.

    // 0 for default
    // -45 for collection
    // +25 just because. We can add more setpoints if necessary.
    controller.povUp().whileTrue(superstructure.setIntakePivotAngle(Degrees.of(25)));
    controller.povRight().whileTrue(superstructure.setIntakePivotAngle(Degrees.of(0)));
    controller.povDown().whileTrue(superstructure.setIntakePivotAngle(Degrees.of(-45)));
  }
}
