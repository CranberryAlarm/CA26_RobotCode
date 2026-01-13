package frc.robot.controls;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;

public class OperatorControls {

  public static void configure(int port, SwerveSubsystem drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    // Intake controls - A to intake, B to eject
    // controller.a().whileTrue(superstructure.intakeCommand());
    // controller.b().whileTrue(superstructure.ejectCommand());

    // Hopper controls - X to run hopper forward, Y to run backward
    // controller.x().whileTrue(superstructure.hopperFeedCommand());
    // controller.y().whileTrue(superstructure.hopperReverseCommand());

    // Shooter controls - Right bumper to shoot
    // controller.rightBumper().whileTrue(superstructure.shootCommand());
    // controller.leftBumper().whileTrue(superstructure.stopShootingCommand());

    // Kicker controls
    // controller.back().whileTrue(superstructure.kickerFeedCommand());
    // controller.start().whileTrue(superstructure.kickerStopCommand());

    // Intake pivot controls
    // controller.povUp().onTrue(superstructure.setIntakeStow());
    // controller.povRight().onTrue(superstructure.setIntakeHold());
    // controller.povRight().onTrue(superstructure.setIntakeDeployed());

    // REAL CONTROLS
    controller.start().onTrue(superstructure.rezeroIntakePivotCommand().ignoringDisable(true));

    controller.rightBumper()
        .whileTrue(superstructure.setIntakeDeployAndRoll().withName("OperatorControls.intakeDeployed"));

    controller.y().whileTrue(superstructure.shootCommand());
    controller.x().whileTrue(superstructure.stopShootingCommand());

    controller.a().whileTrue(
        superstructure.feedAllCommand()
            .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));
  }
}
