package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootOnTheMoveCommand extends Command {
  private final SwerveSubsystem drivetrain;
  private final Superstructure superstructure;

  private Supplier<Translation3d> aimPointSupplier; // The point to aim at
  private AngularVelocity latestShootSpeed;
  private Angle latestHoodAngle;
  private Angle latestTurretAngle;

  public ShootOnTheMoveCommand(SwerveSubsystem drivetrain, Superstructure superstructure,
      Supplier<Translation3d> aimPointSupplier) {
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.aimPointSupplier = aimPointSupplier;

    // We use the drivetrain to determine linear velocity, but don't require it for
    // control. We
    // will be using the superstructure to control the shooting mechanism so it's a
    // requirement.
    // addRequirements(superstructure);

    // TODO: figure out if the above is actually required. Right now, when you start
    // some other command, the auto aim can't start back up again
  }

  @Override
  public void initialize() {
    super.initialize();

    latestHoodAngle = superstructure.getHoodAngle();
    latestTurretAngle = superstructure.getTurretAngle();
    latestShootSpeed = superstructure.getShooterSpeed();

    // TODO: when this current command ends, we should probably cancel the dynamic
    // aim command
    superstructure.aimDynamicCommand(
        () -> {
          return this.latestShootSpeed;
        },
        () -> {
          return this.latestTurretAngle;
        },
        () -> {
          return this.latestHoodAngle;
        }).schedule();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {
    // Calculate trajectory to aimPoint
    var target = aimPointSupplier.get();

    var shooterLocation = drivetrain.getPose3d().getTranslation()
        .plus(superstructure.getShooterPose().getTranslation());

    // Project to ground plane for 2D calculations
    var shooterOnGround = ShootingCalculations.projectToGround(shooterLocation);
    var targetOnGround = ShootingCalculations.projectToGround(target);

    var distanceToTarget = Meters.of(shooterOnGround.getDistance(targetOnGround));

    // Get time of flight using utility class
    double timeOfFlight = ShootingCalculations.getFlightTime(distanceToTarget);

    // Calculate corrective vector based on our current velocity multiplied by time
    // of flight.
    var correctiveVector3d = ShootingCalculations.calculateCorrectiveVector3d(
        drivetrain.getFieldVelocity(), timeOfFlight);

    Logger.recordOutput("FieldSimulation/AimTargetCorrected",
        new Pose3d(target.plus(correctiveVector3d), Rotation3d.kZero));

    var correctedTarget = targetOnGround.plus(
        new Translation2d(correctiveVector3d.getX(), correctiveVector3d.getY()));

    var vectorToTarget = drivetrain.getPose().getTranslation().minus(correctedTarget);

    var correctedDistance = Meters.of(vectorToTarget.getNorm());
    var calculatedHeading = vectorToTarget.getAngle()
        .rotateBy(drivetrain.getHeading().unaryMinus())
        .getMeasure();

    Logger.recordOutput("ShootOnTheMove/RobotHeading", drivetrain.getHeading());
    Logger.recordOutput("ShootOnTheMove/CalculatedHeading", calculatedHeading);
    Logger.recordOutput("ShootOnTheMove/distanceToTarget", distanceToTarget);

    latestTurretAngle = calculatedHeading;
    latestShootSpeed = ShootingCalculations.calculateRequiredShooterSpeed(correctedDistance);

    // TODO: add this back if/when we have a real hood, for now, just set it to the
    // current angle
    // latestHoodAngle =
    // ShootingCalculations.calculateRequiredHoodAngle(correctedDistance);
    latestHoodAngle = superstructure.getHoodAngle();

    superstructure.setShooterSetpoints(
        latestShootSpeed,
        latestTurretAngle,
        latestHoodAngle);

    // System.out.println("Shooting at distance: " + correctedDistance + " requires
    // speed: " + latestShootSpeed
    // + ", hood angle: " + latestHoodAngle + ", turret angle: " +
    // latestTurretAngle);
  }
}
