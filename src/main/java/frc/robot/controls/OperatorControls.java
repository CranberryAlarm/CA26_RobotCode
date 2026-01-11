package frc.robot.controls;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;

public class OperatorControls {
  public static final boolean MACOS_WEIRD_CONTROLLER = true;
  public static void configure(int port, SwerveSubsystem drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    if (Robot.isSimulation()) {
      controller.leftBumper().whileTrue(aimCommand(drivetrain, superstructure));

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
  }

  private static Command aimCommand(SwerveSubsystem drivetrain, Superstructure superstructure) {
    return new ShootOnTheMoveCommand(drivetrain, superstructure, () -> superstructure.getAimPoint());
  }
}
