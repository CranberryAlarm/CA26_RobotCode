package frc.robot.controls;

import static edu.wpi.first.units.Units.Feet;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.RebuildFuelOnFly;

public class OperatorControls {
  public static final boolean MACOS_WEIRD_CONTROLLER = true;

  public static void configure(int port, SwerveSubsystem drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    if (Robot.isSimulation()) {
      controller.leftBumper().whileTrue(aimCommand(drivetrain, superstructure));
      controller.start().whileTrue(fireAlgae(drivetrain, superstructure));

      Commands.run(() -> {
        double leftX = controller.getLeftX();
        double leftY = controller.getLeftY();
        double rightY = controller.getRightY();

        if (MACOS_WEIRD_CONTROLLER) {
          rightY = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
        }

        // Apply deadband
        if (Math.abs(leftX) < Constants.ControllerConstants.DEADBAND)
          leftX = 0;
        if (Math.abs(leftY) < Constants.ControllerConstants.DEADBAND)
          leftY = 0;
        if (Math.abs(rightY) < Constants.ControllerConstants.DEADBAND)
          rightY = 0;

        Translation3d translation = new Translation3d(leftX, leftY, rightY);

        if (MACOS_WEIRD_CONTROLLER) {
          // MacOS Xbox controller mapping is weird - swap X and Y
          translation = new Translation3d(leftY, leftX, rightY);
        }

        // System.out.println("Adjusting pose by: " + translation.toString());

        var newAimPoint = superstructure.getAimPoint().plus(translation.times(0.05));
        // new Transform3d(leftX * 0.05, leftY * 0.05, rightY * 0.05));

        superstructure.setAimPoint(newAimPoint);
      }).ignoringDisable(true).schedule();
    }

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

  private static Command aimCommand(SwerveSubsystem drivetrain, Superstructure superstructure) {
    return new ShootOnTheMoveCommand(drivetrain, superstructure, () -> superstructure.getAimPoint());
  }

  public static Command fireAlgae(SwerveSubsystem drivetrain, Superstructure superstructure) {
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

      GamePieceProjectile fuel = new RebuildFuelOnFly(
          drivetrain.getPose().getTranslation(),
          new Translation2d(),
          drivetrain.getSwerveDrive().getRobotVelocity().times(-1),
          superstructure.getAimRotation3d().toRotation2d(),
          Distance.ofBaseUnits(1, Feet),

          // based on numbers from https://www.reca.lc/flywheel
          superstructure.getTangentialVelocity().times(0.2), // adjust for simulation tuning
          superstructure.getHoodAngle());

      // Configure callbacks to visualize the flight trajectory of the projectile
      fuel.withProjectileTrajectoryDisplayCallBack(
          // Callback for when the note will eventually hit the target (if configured)
          (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileSuccessfulShot",
              pose3ds.toArray(Pose3d[]::new)),
          // Callback for when the note will eventually miss the target, or if no target
          // is configured
          (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileUnsuccessfulShot",
              pose3ds.toArray(Pose3d[]::new)));

      arena.addGamePieceProjectile(fuel);
    }).withName("Fire.Fuel");
  }
}
