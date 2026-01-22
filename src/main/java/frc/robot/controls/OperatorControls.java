package frc.robot.controls;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.maplesim.RebuiltFuelOnFly;

public class OperatorControls {
  public static final boolean MACOS_WEIRD_CONTROLLER = true;

  public static void configure(int port, SwerveSubsystem drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    // if (Robot.isSimulation()) {
    // controller.leftBumper().whileTrue(aimCommand(drivetrain, superstructure));
    // controller.start().whileTrue(fireAlgae(drivetrain, superstructure));

    // Commands.run(() -> {
    // double leftX = controller.getLeftX();
    // double leftY = controller.getLeftY();
    // double rightY = controller.getRightY();

    // if (MACOS_WEIRD_CONTROLLER) {
    // rightY = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
    // }

    // // Apply deadband
    // if (Math.abs(leftX) < Constants.ControllerConstants.DEADBAND)
    // leftX = 0;
    // if (Math.abs(leftY) < Constants.ControllerConstants.DEADBAND)
    // leftY = 0;
    // if (Math.abs(rightY) < Constants.ControllerConstants.DEADBAND)
    // rightY = 0;

    // Translation3d translation = new Translation3d(leftX, leftY, rightY);

    // if (MACOS_WEIRD_CONTROLLER) {
    // // MacOS Xbox controller mapping is weird - swap X and Y
    // translation = new Translation3d(leftY, leftX, rightY);
    // }

    // // System.out.println("Adjusting pose by: " + translation.toString());

    // var newAimPoint = superstructure.getAimPoint().plus(translation.times(0.05));
    // // new Transform3d(leftX * 0.05, leftY * 0.05, rightY * 0.05));

    // superstructure.setAimPoint(newAimPoint);
    // }).ignoringDisable(true).schedule();
    // }

    // REAL CONTROLS
    controller.start().onTrue(superstructure.rezeroIntakePivotCommand().ignoringDisable(true));

    controller.rightBumper()
        .whileTrue(superstructure.setIntakeDeployAndRoll().withName("OperatorControls.intakeDeployed"));

    controller.y().onTrue(superstructure.shootCommand());
    controller.x().whileTrue(superstructure.stopShootingCommand());

    controller.a().whileTrue(
        superstructure.feedAllCommand()
            .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));

    controller.b().whileTrue(
        superstructure.backFeedAllCommand()
            .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));

    controller.povUp().onTrue(superstructure.setTurretForward().withName("OperatorControls.setTurretForward"));
    controller.povLeft().onTrue(superstructure.setTurretLeft().withName("OperatorControls.setTurretLeft"));
    controller.povRight().onTrue(superstructure.setTurretRight().withName("OperatorControls.setTurretRight"));

    controller.leftBumper().toggleOnTrue(
        new ShootOnTheMoveCommand(drivetrain, superstructure, () -> superstructure.getAimPoint())
            .ignoringDisable(true)
            .withName("OperatorControls.aimCommand"));
  }
}
