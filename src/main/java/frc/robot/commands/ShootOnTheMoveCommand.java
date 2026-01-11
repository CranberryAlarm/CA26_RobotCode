package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootOnTheMoveCommand extends Command{
  private final SwerveSubsystem drivetrain;
  private final Superstructure superstructure;

  private Supplier<Translation3d> aimPointSupplier; // The point to aim at
  private AngularVelocity latestShootSpeed;
  private Angle latestHoodAngle;
  private Angle latestTurretAngle;

  public ShootOnTheMoveCommand(SwerveSubsystem drivetrain, Superstructure superstructure, Supplier<Translation3d> aimPointSupplier) {
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.aimPointSupplier = aimPointSupplier;

    // We use the drivetrain to determine linear velocity, but don't require it for control. We
    // will be using the superstructure to control the shooting mechanism so it's a requirement.
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
      super.initialize();

      latestHoodAngle = superstructure.getHoodAngle();
      latestTurretAngle = superstructure.getTurretAngle();
      latestShootSpeed = superstructure.getShooterSpeed();

      superstructure.aimDynamicCommand(
      () -> { return this.latestShootSpeed; },
      () -> { return this.latestTurretAngle; },
      () -> { return this.latestHoodAngle; }
    ).schedule();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {
    // Calculate trajectory to aimPoint
    var target = aimPointSupplier.get();
    System.out.println("Aiming at target pose: " + target);
    var targetOnGround = new Translation2d(target.getX(), target.getY());
    System.out.println("Aiming at target pose (2d): " + targetOnGround);
    Logger.recordOutput("Superstructure/TargetPose2d", targetOnGround);

    System.out.println("Robot pose: " + drivetrain.getPose());
    var vectorToTarget = targetOnGround.minus(drivetrain.getPose().getTranslation());

    Logger.recordOutput("Superstructure/VectorToTarget", vectorToTarget);

    var calculatedHeading = drivetrain.getHeading().minus(vectorToTarget.getAngle()).getMeasure();

    System.out.println("Calculated heading: " + calculatedHeading.in(Degrees) + " degrees");

    latestTurretAngle = calculatedHeading;
  }
}
