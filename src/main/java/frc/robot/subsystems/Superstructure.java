package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Superstructure coordinates the shooter, turret, hood, and intake subsystems
 * for unified control during shooting operations.
 */
public class Superstructure extends SubsystemBase {

  public final ShooterSubsystem shooter;
  public final TurretSubsystem turret;
  public final HoodSubsystem hood;
  public final IntakeSubsystem intake;
  public final HopperSubsystem hopper;
  public final KickerSubsystem kicker;

  // Tolerance for "at setpoint" checks
  private static final AngularVelocity SHOOTER_TOLERANCE = RPM.of(100);
  private static final Angle TURRET_TOLERANCE = Degrees.of(1);
  private static final Angle HOOD_TOLERANCE = Degrees.of(2);

  // Triggers for readiness checks
  public final Trigger isShooterAtSpeed;
  public final Trigger isTurretOnTarget;
  public final Trigger isHoodOnTarget;
  public final Trigger isReadyToShoot;

  private AngularVelocity targetShooterSpeed = RPM.of(0);
  private Angle targetTurretAngle = Degrees.of(0);
  private Angle targetHoodAngle = Degrees.of(0);

  // Hard coded red hub aim point
  private Translation3d aimPoint = new Translation3d(Meter.of(11.902), Meter.of(4.031), Meter.of(0));

  public Superstructure(ShooterSubsystem shooter, TurretSubsystem turret, HoodSubsystem hood, IntakeSubsystem intake,
      HopperSubsystem hopper, KickerSubsystem kicker) {
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;
    this.intake = intake;
    this.hopper = hopper;
    this.kicker = kicker;

    // Create triggers for checking if mechanisms are at their targets
    this.isShooterAtSpeed = new Trigger(
        () -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM)) < SHOOTER_TOLERANCE.in(RPM));

    this.isTurretOnTarget = new Trigger(
        () -> Math.abs(turret.getRawAngle().in(Degrees) - targetTurretAngle.in(Degrees)) < TURRET_TOLERANCE
            .in(Degrees));

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
            // shooter.setSpeed(shooterSpeed).asProxy(),
            turret.setAngle(turretAngle).asProxy(),
            hood.setAngle(hoodAngle).asProxy()))
        .withName("Superstructure.aim");
  }

  public void setShooterSetpoints(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    targetShooterSpeed = shooterSpeed;
    targetTurretAngle = turretAngle;
    targetHoodAngle = hoodAngle;
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
    return Commands.parallel(
        shooter.setSpeedDynamic(shooterSpeedSupplier).asProxy(),
        turret.setAngleDynamic(turretAngleSupplier).asProxy(),
        hood.setAngleDynamic(hoodAngleSupplier).asProxy())
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
    return aimDynamicCommand(() -> shooterSpeed, () -> turretAngle, () -> hoodAngle)
        .andThen(waitUntilReadyCommand())
        .withName("Superstructure.aimAndWait");
  }

  public Command setTurretForward() {
    return turret.setAngle(Degrees.of(0)).withName("Superstructure.setTurretForward");
  }

  public Command setTurretLeft() {
    return turret.setAngle(Degrees.of(45)).withName("Superstructure.setTurretLeft");
  }

  public Command setTurretRight() {
    return turret.setAngle(Degrees.of(-45)).withName("Superstructure.setTurretRight");
  }

  // Getters for current state
  public AngularVelocity getShooterSpeed() {
    return shooter.getSpeed();
  }

  public Angle getTurretAngle() {
    return turret.getRawAngle();
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
    // See
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    return new Rotation3d(
        Degrees.of(0), // no roll 🤞
        hood.getAngle().unaryMinus(), // pitch is negative hood angle
        turret.getRobotAdjustedAngle());
  }

  /**
   * Command to run the intake while held.
   */
  public Command intakeCommand() {
    return intake.intakeCommand().withName("Superstructure.intake");
  }

  /**
   * Command to eject while held.
   */
  public Command ejectCommand() {
    return intake.ejectCommand().withName("Superstructure.eject");
  }

  /**
   * Command to run the hopper forward while held.
   */
  public Command hopperFeedCommand() {
    return hopper.feedCommand().withName("Superstructure.feed");
  }

  /**
   * Command to run the hopper in reverse while held.
   */
  public Command hopperReverseCommand() {
    return hopper.reverseCommand().withName("Superstructure.hopperReverse");
  }

  /**
   * Command to run the kicker forward while held, stops when released.
   */
  public Command kickerFeedCommand() {
    return kicker.feedCommand().withName("Superstructure.kickerFeed");
  }

  /**
   * Command to run the kicker stop while held, stops when released.
   */
  public Command kickerStopCommand() {
    return kicker.stopCommand().withName("Superstructure.kickerStop");
  }

  public Command feedAllCommand() {
    return Commands.parallel(
        hopper.feedCommand().asProxy(),
        kicker.feedCommand().asProxy(),
        intake.setPivotAngle(Degrees.of(46)).asProxy()).withName("Superstructure.feedAll");
  }

  public Command backFeedAllCommand() {
    return Commands.parallel(
        hopper.backFeedCommand().asProxy(),
        intake.backFeedAndRollCommand().asProxy()).withName("Superstructure.backFeedAll");
  }

  // public Command intakeBounceCommand() {
  // return Commands.sequence(
  // Commands.runOnce(() -> intake.setPivotAngle(Degrees.of(115))).asProxy()
  // .withName("Superstructure.intakeBounce.deploy"),
  // Commands.waitSeconds(0.5),
  // Commands.runOnce(() -> intake.setPivotAngle(Degrees.of(59))).asProxy()
  // .withName("Superstructure.intakeBounce.feed"),
  // Commands.waitSeconds(0.5))
  // .withName("Superstructure.intakeBounce");
  // }

  public Command stopFeedingAllCommand() {
    return Commands.parallel(
        hopper.stopCommand().asProxy(),
        kicker.stopCommand().asProxy(),
        intake.deployAndRollCommand().asProxy()).withName("Superstructure.stopFeedingAll");
  }

  /**
   * Command to set the intake pivot angle.
   */
  public Command setIntakePivotAngle(Angle angle) {
    return intake.setPivotAngle(angle).withName("Superstructure.setIntakePivotAngle");
  }

  public Command setIntakeDeployAndRoll() {
    return intake.deployAndRollCommand().withName("Superstructure.setIntakeDeployAndRoll");
  }

  /**
   * Command to shoot - spins up shooter.
   */
  public Command shootCommand() {
    // return shooter.sysId().withName("Superstructure.shoot");
    return shooter.spinUp().withName("Superstructure.shoot");
  }

  /**
   * Command to shoot - spins up shooter.
   */
  public Command shootWithDistanceCommand(double distanceMeters) {
    return shooter.shootAtDistance(distanceMeters).withName("Superstructure.shootWithDistance");
  }

  /**
   * Command to stop shooting - stops shooter.
   */
  public Command stopShootingCommand() {
    return shooter.stop().withName("Superstructure.stopShooting");
  }

  // Re-zero intake pivot if needed
  public Command rezeroIntakePivotCommand() {
    return intake.rezero().withName("Superstructure.rezeroIntakePivot");
  }

  @Override
  public void periodic() {
    // Superstructure doesn't need periodic updates - subsystems handle their own

    String shooterOut = "S:" + isShooterAtSpeed.getAsBoolean() + "(" + Math.round(shooter.getSpeed().in(RPM)) + "/"
        + Math.round(targetShooterSpeed.in(RPM)) + ")";

    String turretOut = "T:" + isTurretOnTarget.getAsBoolean() + "(" + Math.round(turret.getRawAngle().in(Degrees)) + "/"
        + Math.round(targetTurretAngle.in(Degrees)) + ")";

    String hoodOut = "H:" + isHoodOnTarget.getAsBoolean() + "(" + Math.round(hood.getAngle().in(Degrees)) + "/"
        + Math.round(targetHoodAngle.in(Degrees)) + ")";

    String readyOut = "R:" + isReadyToShoot.getAsBoolean();

    System.err.println(shooterOut + " " + turretOut + " " + hoodOut + " " + readyOut);
  }

  public Command useRequirement() {
    return runOnce(() -> {
    });
  }

  public Pose3d getShooterPose() {
    // Position of the shooter relative to the "front" of the robot. Rotation
    // element is based on hood and turret angles
    return new Pose3d(new Translation3d(
        Meter.of(-0.3),
        Meter.of(0),
        Meter.of(0.6)),
        getAimRotation3d());
  }

  public LinearVelocity getTangentialVelocity() {
    return shooter.getTangentialVelocity();
  }
}
