package frc.robot.controls;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;

public class OperatorControls {
  public static void configure(int port, SwerveSubsystem drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    if (Robot.isSimulation()) {
      controller.leftBumper().whileTrue(aimCommand(drivetrain, superstructure));

      Commands.run(() -> {
        double leftX = controller.getLeftX();
        double leftY = controller.getLeftY();
        double rightY = controller.getRightY();

        // Apply deadband
        if (Math.abs(leftX) < Constants.ControllerConstants.DEADBAND)
          leftX = 0;
        if (Math.abs(leftY) < Constants.ControllerConstants.DEADBAND)
          leftY = 0;
        if (Math.abs(rightY) < Constants.ControllerConstants.DEADBAND)
          rightY = 0;

        Translation3d translation = new Translation3d(leftX, leftY, rightY);

        // System.out.println("Adjusting pose by: " + translation.toString());
;
        var newAimPoint = superstructure.getAimPoint().plus(translation.times(0.05));
          // new Transform3d(leftX * 0.05, leftY * 0.05, rightY * 0.05));

        superstructure.setAimPoint(newAimPoint);
      }).ignoringDisable(true).schedule();
    }
  }


  private static Command aimCommand(SwerveSubsystem drivetrain, Superstructure superstructure) {
    return superstructure.aimDynamicCommand(() -> { return RotationsPerSecond.of(100); }, () -> {
      var target = superstructure.getAimPoint();
      System.out.println("Aiming at target pose: " + target);
      var targetOnGround = new Translation2d(target.getX(), target.getY());
      System.out.println("Aiming at target pose (2d): " + targetOnGround);
      Logger.recordOutput("Superstructure/TargetPose2d", targetOnGround);

      System.out.println("Robot pose: " + drivetrain.getPose());
      var vectorToTarget = targetOnGround.minus(drivetrain.getPose().getTranslation());

      // Calculate relative direction to target relative to the front of the robot
      var frontOfRobot = drivetrain.getHeading();
      var robotToTarget = vectorToTarget.rotateBy(frontOfRobot.unaryMinus());
      return robotToTarget.getAngle().getMeasure();
      }, () -> {
        return Degrees.of(45); // Placeholder value
      });
  }
}
