package frc.robot.controls;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class DriverControls {

  private static Pose2d getTargetPose() {
    return PoseControls.getTargetPose();
  }

  public static void configure(int port, SwerveSubsystem drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(ControllerConstants.kDriverControllerPort);

    SwerveInputStream driveInputStream = SwerveInputStream.of(drivetrain.getSwerveDrive(),
        () -> controller.getLeftY() * -1,
        () -> controller.getLeftX() * -1)
        .withControllerRotationAxis(() -> controller.getRightX() * -1)
        .robotRelative(false)
        .allianceRelativeControl(true)
        // .scaleTranslation(0.8) // TODO: Tune speed scaling
        .deadband(ControllerConstants.DEADBAND);

    controller.y().whileTrue(Commands.run(
        () -> {
          driveInputStream
              .aim(getTargetPose())
              .aimWhile(true);
        }).finallyDo(() -> driveInputStream.aimWhile(false)));

    drivetrain.setDefaultCommand(
        drivetrain.driveFieldOriented(driveInputStream).withName("Drive" + ".test"));

    // NOTE: YAGSL way of doing direct drive to pose
    // driveInputStream.driveToPose(drivetrain.getTargetPoseSupplier(),
    // new ProfiledPIDController(5, 0, 0,
    // new Constraints(5, 2)),
    // new ProfiledPIDController(5, 0, 0,
    // new Constraints(
    // Units.degreesToRadians(360),
    // Units.degreesToRadians(180))));

    // controller.rightBumper().whileTrue(Commands.runEnd(
    // () -> driveInputStream.driveToPoseEnabled(true),
    // () -> driveInputStream.driveToPoseEnabled(false)));

    // NOTE: PathPlanner way of doing obstacle-aware drive to pose
    controller.rightBumper()
        .whileTrue(Commands.defer(
            () -> drivetrain.driveToPose(drivetrain.getTargetPose()),
            java.util.Set.of(drivetrain)));

    if (DriverStation.isTest()) {
      // drivetrain.setDefaultCommand(driveFieldOrientedAngularVelocity);
      // Overrides drive command above!
      // Might be useful for robot-oriented controls in testing

      controller.a().onTrue((Commands.runOnce(drivetrain::zeroGyro)));
      controller.b().whileTrue(drivetrain.centerModulesCommand());
      controller.x().whileTrue(Commands.runOnce(drivetrain::lock, drivetrain).repeatedly());
      controller.y().onTrue((Commands.runOnce(drivetrain::zeroGyro)));

      controller.start().whileTrue(drivetrain.sysIdAngleMotorCommand());
      controller.back().whileTrue(drivetrain.sysIdDriveMotorCommand());

      controller.leftBumper().onTrue(Commands.none());
    } else if (Robot.isSimulation()) {
      controller.back().whileTrue(fireAlgae(drivetrain));
    } else {
      controller.leftBumper().whileTrue(Commands.runOnce(drivetrain::lock, drivetrain).repeatedly());
    }
  }

  public static Command fireAlgae(SwerveSubsystem drivetrain) {
    return Commands.runOnce(() -> {
      System.err.println("FIRE!");

      SimulatedArena arena = SimulatedArena.getInstance();

      // Translation2d robotPosition,
      // Translation2d shooterPositionOnRobot,
      // ChassisSpeeds chassisSpeeds,
      // Rotation2d shooterFacing,
      // Distance initialHeight,
      // LinearVelocity launchingSpeed,
      // Angle shooterAngle

      ReefscapeAlgaeOnFly algae = new ReefscapeAlgaeOnFly(
          drivetrain.getPose().getTranslation(),
          new Translation2d(),
          drivetrain.getSwerveDrive().getRobotVelocity().times(-1),
          drivetrain.getSwerveDrive().getOdometryHeading(),
          Distance.ofBaseUnits(1, Feet),
          LinearVelocity.ofBaseUnits(5, FeetPerSecond),
          Angle.ofBaseUnits(45, Degrees));

      // Configure callbacks to visualize the flight trajectory of the projectile
      algae.withProjectileTrajectoryDisplayCallBack(
          // Callback for when the note will eventually hit the target (if configured)
          (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileSuccessfulShot",
              pose3ds.toArray(Pose3d[]::new)),
          // Callback for when the note will eventually miss the target, or if no target
          // is configured
          (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileUnsuccessfulShot",
              pose3ds.toArray(Pose3d[]::new)));

      arena.addGamePieceProjectile(algae);
    }).withName("Fire.Algae");
  }
}
