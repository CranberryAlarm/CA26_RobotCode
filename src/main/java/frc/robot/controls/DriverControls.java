package frc.robot.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class DriverControls {
  public enum DriveMode {
    ROBOT_ORIENTED,
    FIELD_ORIENTED,
    FIELD_ORIENTED_HEADING,
    AIM_AT_POSE
  }

  private static DriveMode driveMode = DriveMode.FIELD_ORIENTED_HEADING;

  public void setDriveMode(DriveMode mode) {
    driveMode = mode;
  }

  public static void configure(int port, SwerveSubsystem drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(ControllerConstants.kDriverControllerPort);

    SwerveInputStream driveInputStream = SwerveInputStream.of(drivetrain.getSwerveDrive(),
        () -> controller.getLeftY() * -1,
        () -> controller.getLeftX() * -1)
        // .scaleTranslation(0.8) // TODO: Tune speed scaling
        .deadband(ControllerConstants.DEADBAND);

    switch (driveMode) {
      case ROBOT_ORIENTED -> driveInputStream
          .robotRelative(true)
          .allianceRelativeControl(false)
          .withControllerRotationAxis(() -> controller.getRightX() * -1);
      case FIELD_ORIENTED -> driveInputStream
          .robotRelative(false)
          .allianceRelativeControl(true)
          .withControllerRotationAxis(() -> controller.getRightX() * -1)
          .headingWhile(true);
      case FIELD_ORIENTED_HEADING -> driveInputStream
          .robotRelative(false)
          .allianceRelativeControl(true)
          .withControllerHeadingAxis(
              controller::getRightX,
              controller::getRightY)
          .headingWhile(true)
          .translationHeadingOffset(true)
          .translationHeadingOffset(Rotation2d.fromDegrees(0));
      case AIM_AT_POSE -> {
        // ???
      }
    }

    drivetrain.setDefaultCommand(
        drivetrain.driveFieldOriented(driveInputStream).withName("Drive" + driveMode.name()));

    // if (Robot.isSimulation()) {
    // Pose2d target = new Pose2d(new Translation2d(1, 4),
    // Rotation2d.fromDegrees(90));
    // drivetrain.getSwerveDrive().field.getObject("targetPose").setPose(target);

    // driveDirectAngleKeyboard.driveToPose(() -> target,
    // new ProfiledPIDController(5, 0, 0,
    // new Constraints(5, 2)),
    // new ProfiledPIDController(5, 0, 0,
    // new Constraints(
    // Units.degreesToRadians(360),
    // Units.degreesToRadians(180))));
    // controller.start().onTrue(Commands.runOnce(() -> drivetrain.resetOdometry(new
    // Pose2d(3, 3, new Rotation2d()))));
    // controller.button(1).whileTrue(drivetrain.sysIdDriveMotorCommand());
    // controller.button(2).whileTrue(Commands.runEnd(
    // () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
    // () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
    // }

    if (DriverStation.isTest()) {
      // drivetrain.setDefaultCommand(driveFieldOrientedAngularVelocity); // Overrides
      // drive command above!

      controller.x().whileTrue(Commands.runOnce(drivetrain::lock, drivetrain).repeatedly());
      controller.start().onTrue((Commands.runOnce(drivetrain::zeroGyro)));
      controller.back().whileTrue(drivetrain.centerModulesCommand());
      controller.leftBumper().onTrue(Commands.none());
      controller.rightBumper().onTrue(Commands.none());
    } else {
      controller.a().onTrue((Commands.runOnce(drivetrain::zeroGyro)));

      controller.start().whileTrue(drivetrain.sysIdAngleMotorCommand());
      controller.back().whileTrue(drivetrain.sysIdDriveMotorCommand());
      // controller.back().whileTrue(fireAlgae());

      controller.leftBumper().whileTrue(Commands.runOnce(drivetrain::lock, drivetrain).repeatedly());
      controller.rightBumper().onTrue(Commands.none());
    }
  }
}
