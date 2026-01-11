package frc.robot.subsystems;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Superstructure coordinates the shooter, turret, and hood subsystems
 * for unified control during shooting operations.
 */
public class Superstructure extends SubsystemBase {

  private final ShooterSubsystem shooter;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;

  // Default values for "ready" state
  private static final AngularVelocity DEFAULT_SHOOTER_SPEED = RPM.of(4000);
  private static final Angle DEFAULT_TURRET_ANGLE = Degrees.of(0);
  private static final Angle DEFAULT_HOOD_ANGLE = Degrees.of(45);

  // Tolerance for "at setpoint" checks
  private static final AngularVelocity SHOOTER_TOLERANCE = RPM.of(100);
  private static final Angle TURRET_TOLERANCE = Degrees.of(2);
  private static final Angle HOOD_TOLERANCE = Degrees.of(2);

  // Triggers for readiness checks
  public final Trigger isShooterAtSpeed;
  public final Trigger isTurretOnTarget;
  public final Trigger isHoodOnTarget;
  public final Trigger isReadyToShoot;

  private AngularVelocity targetShooterSpeed = RPM.of(0);
  private Angle targetTurretAngle = Degrees.of(0);
  private Angle targetHoodAngle = Degrees.of(0);

  private Translation3d aimPoint = new Translation3d(Meters.of(0), Meters.of(0), Meters.of(0));

  public Superstructure(ShooterSubsystem shooter, TurretSubsystem turret, HoodSubsystem hood) {
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;

    this.aimPoint = new Translation3d(Meters.of(14.5), Meters.of(4), Meters.of(0));

    // Create triggers for checking if mechanisms are at their targets
    this.isShooterAtSpeed = new Trigger(
        () -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM)) < SHOOTER_TOLERANCE.in(RPM));

    this.isTurretOnTarget = new Trigger(
        () -> Math.abs(turret.getAngle().in(Degrees) - targetTurretAngle.in(Degrees)) < TURRET_TOLERANCE.in(Degrees));

    this.isHoodOnTarget = new Trigger(
        () -> Math.abs(hood.getAngle().in(Degrees) - targetHoodAngle.in(Degrees)) < HOOD_TOLERANCE.in(Degrees));

    this.isReadyToShoot = isShooterAtSpeed.and(isTurretOnTarget).and(isHoodOnTarget);
  }

  /**
   * Stops all mechanisms - shooter stops spinning, turret and hood hold position.
   */
  public Command stopAllCommand() {
    return Commands.parallel(
        shooter.stop().asProxy(),
        turret.set(0).asProxy(),
        hood.set(0).asProxy()).withName("Superstructure.stopAll");
  }

  /**
   * Moves all mechanisms to a default "ready" state:
   * - Shooter spun up to default speed
   * - Turret centered
   * - Hood at 45 degrees
   */
  public Command readyCommand() {
    return Commands.runOnce(() -> {
      targetShooterSpeed = DEFAULT_SHOOTER_SPEED;
      targetTurretAngle = DEFAULT_TURRET_ANGLE;
      targetHoodAngle = DEFAULT_HOOD_ANGLE;
    }).andThen(
        Commands.parallel(
            shooter.setSpeed(DEFAULT_SHOOTER_SPEED).asProxy(),
            turret.center().asProxy(),
            hood.setAngle(DEFAULT_HOOD_ANGLE).asProxy()))
        .withName("Superstructure.ready");
  }

  /**
   * Stows the superstructure - stops shooter, centers turret, stows hood.
   */
  public Command stowCommand() {
    return Commands.runOnce(() -> {
      targetShooterSpeed = RPM.of(0);
      targetTurretAngle = Degrees.of(0);
      targetHoodAngle = Degrees.of(0);
    }).andThen(
        Commands.parallel(
            shooter.stop().asProxy(),
            turret.center().asProxy(),
            hood.stow().asProxy()))
        .withName("Superstructure.stow");
  }

  /**
   * Aims the superstructure to specific targets - used for auto-targeting.
   *
   * @param shooterSpeed Target shooter speed
   * @param turretAngle  Target turret angle
   * @param hoodAngle    Target hood angle
   */
  public Command aimCommand(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    return Commands.runOnce(() -> {
      targetShooterSpeed = shooterSpeed;
      targetTurretAngle = turretAngle;
      targetHoodAngle = hoodAngle;
    }).andThen(
        Commands.parallel(
            shooter.setSpeed(shooterSpeed).asProxy(),
            turret.setAngle(turretAngle).asProxy(),
            hood.setAngle(hoodAngle).asProxy()))
        .withName("Superstructure.aim");
  }

  /**
   * Aims the superstructure using suppliers - useful for dynamic targeting.
   *
   * @param shooterSpeedSupplier Supplier for target shooter speed
   * @param turretAngleSupplier  Supplier for target turret angle
   * @param hoodAngleSupplier    Supplier for target hood angle
   */
  public Command aimDynamicCommand(
      Supplier<AngularVelocity> shooterSpeedSupplier,
      Supplier<Angle> turretAngleSupplier,
      Supplier<Angle> hoodAngleSupplier) {
    return Commands.run(() -> {
      targetShooterSpeed = shooterSpeedSupplier.get();
      targetTurretAngle = turretAngleSupplier.get();
      targetHoodAngle = hoodAngleSupplier.get();
    }).alongWith(
        Commands.parallel(
            shooter.setSpeedDynamic(shooterSpeedSupplier).asProxy(),
            turret.setAngleDynamic(turretAngleSupplier).asProxy(),
            hood.setAngleDynamic(hoodAngleSupplier).asProxy()))
        .withName("Superstructure.aimDynamic");
  }

  /**
   * Waits until the superstructure is ready to shoot.
   */
  public Command waitUntilReadyCommand() {
    return Commands.waitUntil(isReadyToShoot).withName("Superstructure.waitUntilReady");
  }

  /**
   * Aims and waits until ready - combines aim and wait.
   */
  public Command aimAndWaitCommand(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    return aimCommand(shooterSpeed, turretAngle, hoodAngle)
        .andThen(waitUntilReadyCommand())
        .withName("Superstructure.aimAndWait");
  }

  // Getters for current state
  public AngularVelocity getShooterSpeed() {
    return shooter.getSpeed();
  }

  public Angle getTurretAngle() {
    return turret.getAngle();
  }

  public Angle getHoodAngle() {
    return hood.getAngle();
  }

  public AngularVelocity getTargetShooterSpeed() {
    return targetShooterSpeed;
  }

  public Angle getTargetTurretAngle() {
    return targetTurretAngle;
  }

  public Angle getTargetHoodAngle() {
    return targetHoodAngle;
  }

  public Translation3d getAimPoint() {
    return aimPoint;
  }

  public void setAimPoint(Translation3d newAimPoint) {
    this.aimPoint = newAimPoint;
  }

  public Rotation3d getAimRotation3d() {
    // See https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    return new Rotation3d(
      Degrees.of(0), // no roll 🤞
      hood.getAngle().unaryMinus(), // pitch is negative hood angle
      turret.getAngle().unaryMinus()); // yaw is also negative
  }

  @Override
  public void periodic() {
    // Superstructure doesn't need periodic updates - subsystems handle their own
  }

  public Pose3d getShooterPose() {
    // Position of the shooter relative to the "front" of the robot. Rotation element is based on hood and turret angles
    return new Pose3d(Meters.of(0), Meters.of(0), Meters.of(0), getAimRotation3d());
  }
}
