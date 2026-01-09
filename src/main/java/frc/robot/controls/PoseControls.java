package frc.robot.controls;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Controls for moving a virtual target pose on the field.
 * The target pose can be used for autonomous navigation (drive-to-pose).
 */
public class PoseControls {
  private static Pose2d targetPose = new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0));

  // Movement speeds for adjusting target pose
  private static final double TRANSLATION_SPEED = 0.05; // meters per periodic cycle
  private static final double ROTATION_SPEED = 2.0; // degrees per periodic cycle

  /**
   * Configure pose controls on the given controller.
   *
   * @param controller The controller to bind pose adjustment controls to
   * @param drivetrain The swerve subsystem for updating the field visualization
   */
  public static void configure(int port, SwerveSubsystem drivetrain) {
    CommandXboxController controller = new CommandXboxController(port);

    // Update the field visualization with initial pose
    updateFieldPose(drivetrain);

    // Continuous joystick control for target pose adjustment
    // Left stick: X/Y translation, Right stick X: Rotation
    // Using ignoringDisable(true) so it runs even when disabled, and no
    // requirements so it doesn't conflict
    Commands.run(() -> {
      double leftX = controller.getLeftX();
      double leftY = controller.getLeftY();
      double rightX = controller.getRightX();

      // Apply deadband
      if (Math.abs(leftX) < Constants.ControllerConstants.DEADBAND)
        leftX = 0;
      if (Math.abs(leftY) < Constants.ControllerConstants.DEADBAND)
        leftY = 0;
      if (Math.abs(rightX) < Constants.ControllerConstants.DEADBAND)
        rightX = 0;

      // Alliance-relative controls:
      // Forward (negative leftY) should move away from driver station
      // Blue alliance: driver station at x=0, so forward = +X
      // Red alliance: driver station at x=16.5m, so forward = -X (flip X direction)
      // Left (negative leftX) should move toward left side of field from driver POV
      // Blue alliance: left = +Y
      // Red alliance: left = -Y (flip Y direction)
      boolean isRedAlliance = DriverStation.getAlliance()
          .map(alliance -> alliance == DriverStation.Alliance.Red)
          .orElse(false);

      double allianceMultiplier = isRedAlliance ? -1.0 : 1.0;

      // Adjust target pose based on joystick input (alliance-relative)
      if (leftX != 0 || leftY != 0) {
        // -leftY = forward motion, leftX = strafe right
        // Apply alliance multiplier to make controls relative to driver station
        adjustTargetTranslation(
            -leftY * TRANSLATION_SPEED * allianceMultiplier, // Forward/back -> X
            -leftX * TRANSLATION_SPEED * allianceMultiplier); // Left/right -> Y
      }
      if (rightX != 0) {
        adjustTargetRotation(-rightX * ROTATION_SPEED);
      }

      updateFieldPose(drivetrain);
    }).ignoringDisable(true).withName("PoseControls.Update").schedule();

    // Y button: Reset target pose to robot's current position
    controller.y().onTrue(Commands.runOnce(() -> {
      targetPose = drivetrain.getPose();
      updateFieldPose(drivetrain);
    }).ignoringDisable(true));
  }

  /**
   * Adjust the target translation by the given delta values.
   *
   * @param deltaX Change in X position (meters)
   * @param deltaY Change in Y position (meters)
   */
  private static void adjustTargetTranslation(double deltaX, double deltaY) {
    targetPose = new Pose2d(
        targetPose.getX() + deltaX,
        targetPose.getY() + deltaY,
        targetPose.getRotation());
  }

  /**
   * Adjust the target rotation by the given delta.
   *
   * @param deltaDegrees Change in rotation (degrees)
   */
  private static void adjustTargetRotation(double deltaDegrees) {
    targetPose = new Pose2d(
        targetPose.getTranslation(),
        targetPose.getRotation().plus(Rotation2d.fromDegrees(deltaDegrees)));
  }

  /**
   * Update the field visualization with the current target pose.
   *
   * @param drivetrain The swerve subsystem containing the field
   */
  private static void updateFieldPose(SwerveSubsystem drivetrain) {
    drivetrain.getSwerveDrive().field.getObject("targetPose").setPose(targetPose);
  }

  /**
   * Get the current target pose.
   *
   * @return The current target pose
   */
  public static Pose2d getTargetPose() {
    return targetPose;
  }

  /**
   * Get a supplier for the current target pose.
   * Useful for commands that need a dynamic pose reference.
   *
   * @return Supplier that returns the current target pose
   */
  public static Supplier<Pose2d> getTargetPoseSupplier() {
    return () -> targetPose;
  }

  /**
   * Set the target pose directly.
   *
   * @param pose The new target pose
   */
  public static void setTargetPose(Pose2d pose) {
    targetPose = pose;
  }

  /**
   * Set the target pose and update the field visualization.
   *
   * @param pose       The new target pose
   * @param drivetrain The swerve subsystem for updating the field
   */
  public static void setTargetPose(Pose2d pose, SwerveSubsystem drivetrain) {
    targetPose = pose;
    updateFieldPose(drivetrain);
  }
}
