package frc.robot.subsystems;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Superstructure coordinates the shooter, turret, hood, and intake subsystems
 * for unified control during shooting operations.
 */
public class Superstructure extends SubsystemBase {

  private final ShooterSubsystem shooter;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;
  private final IntakeSubsystem intake;
  private final HopperSubsystem hopper;
  private final KickerSubsystem kicker;

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

  public Superstructure(ShooterSubsystem shooter, TurretSubsystem turret, HoodSubsystem hood, IntakeSubsystem intake,
      HopperSubsystem hopper, KickerSubsystem kicker) {
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;
    this.intake = intake;
    this.hopper = hopper;
    this.kicker = kicker;

    // Create triggers for checking if mechanisms are at their targets
    this.isShooterAtSpeed = new Trigger(() -> false);
    // this.isShooterAtSpeed = new Trigger(
    // () -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM)) <
    // SHOOTER_TOLERANCE.in(RPM));

    this.isTurretOnTarget = new Trigger(
        () -> Math.abs(turret.getAngle().in(Degrees) - targetTurretAngle.in(Degrees)) < TURRET_TOLERANCE.in(Degrees));

    this.isHoodOnTarget = new Trigger(
        () -> Math.abs(hood.getAngle().in(Degrees) - targetHoodAngle.in(Degrees)) < HOOD_TOLERANCE.in(Degrees));

    this.isReadyToShoot = new Trigger(() -> false);
    // this.isReadyToShoot =
    // isShooterAtSpeed.and(isTurretOnTarget).and(isHoodOnTarget);
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
            // shooter.setSpeed(DEFAULT_SHOOTER_SPEED).asProxy(),
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
            // shooter.stop().asProxy(),
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
            // shooter.setSpeed(shooterSpeed).asProxy(),
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
            // shooter.setSpeed(shooterSpeedSupplier.get()).asProxy(),
            turret.setAngle(turretAngleSupplier.get()).asProxy(),
            hood.setAngle(hoodAngleSupplier.get()).asProxy()))
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
  // public AngularVelocity getShooterSpeed() {
  // return shooter.getSpeed();
  // }

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
    return shooter.spinUp().withName("Superstructure.shoot");
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
  }
}
