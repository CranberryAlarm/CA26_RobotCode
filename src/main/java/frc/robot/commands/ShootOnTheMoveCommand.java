package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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

    var shooterLocation = drivetrain.getPose3d().getTranslation().plus(superstructure.getShooterPose().getTranslation());

    // Ignore this parameter for now, the range tables will account for it :/
    // var deltaH = target.getMeasureZ().minus(shooterLocation.getMeasureZ());
    var shooterOnGround = new Translation2d(shooterLocation.getX(), shooterLocation.getY());
    var targetOnGround = new Translation2d(target.getX(), target.getY());

    var distanceToTarget = Meters.of(shooterOnGround.getDistance(targetOnGround));

    // Get time of flight. We could try to do this analytically but for now it's easier and more realistic
    // to use a simple linear approximation based on empirical data.
    double timeOfFlight = getFlightTime(distanceToTarget);

    // Calculate corrective vector based on our current velocity multiplied by time of flight.
    // If we're stationary, this should be zero. If we're backing up, this will be "ahead" of the target, etc.
    var updatedPosition = drivetrain.getFieldVelocity().times(timeOfFlight);
    var correctiveVector = new Translation2d(updatedPosition.vxMetersPerSecond, updatedPosition.vyMetersPerSecond).unaryMinus();
    var correctiveVector3d = new Translation3d(updatedPosition.vxMetersPerSecond, updatedPosition.vyMetersPerSecond, 0);

    Logger.recordOutput("FieldSimulation/AimTargetCorrected", new Pose3d(target.plus(correctiveVector3d), Rotation3d.kZero));

    var correctedTarget = targetOnGround.plus(correctiveVector);

    var vectorToTarget = drivetrain.getPose().getTranslation().minus(correctedTarget);

    var correctedDistance = Meters.of(vectorToTarget.getNorm());
    var calculatedHeading = vectorToTarget.getAngle().rotateBy(drivetrain.getHeading()).getMeasure();

    Logger.recordOutput("ShootOnTheMove/RobotHeading", drivetrain.getHeading());
    Logger.recordOutput("ShootOnTheMove/CalculatedHeading", calculatedHeading);

    latestTurretAngle = calculatedHeading;
    latestShootSpeed = calculateRequiredShooterSpeed(correctedDistance);
    latestHoodAngle = calculateRequiredHoodAngle(correctedDistance);
  }

  private double getFlightTime(Distance distanceToTarget) {
    // Simple linear approximation based on empirical data.
    return TIME_OF_FLIGHT_BY_DISTANCE.get(distanceToTarget.in(Meters));
  }

  private AngularVelocity calculateRequiredShooterSpeed(Distance distanceToTarget) {
    return RotationsPerSecond.of(SHOOTING_SPEED_BY_DISTANCE.get(distanceToTarget.in(Meters)));
  }

  private Angle calculateRequiredHoodAngle(Distance distanceToTarget) {
    return Degrees.of(HOOD_ANGLE_BY_DISTANCE.get(distanceToTarget.in(Meters)));
  }

  // meters, seconds
  private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
    Map.entry(1.0, 3.0),
    Map.entry(2.0, 4.0),
    Map.entry(3.0, 5.0)
  );

  // meters, RPS
  private static final InterpolatingDoubleTreeMap SHOOTING_SPEED_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
    Map.entry(1.0, 100.0),
    Map.entry(2.0, 100.0),
    Map.entry(3.0, 205.0)
  );

  // meters, degrees
  private static final InterpolatingDoubleTreeMap HOOD_ANGLE_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
    Map.entry(1.0, 15.0),
    Map.entry(2.0, 30.0),
    Map.entry(3.0, 45.0)
  );
}
