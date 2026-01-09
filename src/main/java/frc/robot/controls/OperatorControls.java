package frc.robot.controls;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;

public class OperatorControls {

  public static void configure(int port, SwerveSubsystem drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    if (Robot.isSimulation() && superstructure != null) {
      controller.leftTrigger().whileTrue(superstructure.aimDynamicCommand(() -> { return RotationsPerSecond.of(100); }, () -> {
        // Some combination of these is what I want.
        return Degrees.of(drivetrain.getPose().relativeTo(superstructure.getTargetPose().toPose2d()).getRotation().getDegrees());
      }, () -> {
        return Degrees.of(45); // Placeholder value
      }));
    }
  }
}
