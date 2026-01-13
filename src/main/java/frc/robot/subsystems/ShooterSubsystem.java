package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.thethriftybot.ThriftyNova;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private double SHOOTER_SPEED = 0.65;
  private double temp_distance = 0.0;

  // 2 Neos, 4in shooter wheels
  private final ThriftyNova leaderNova = new ThriftyNova(Constants.ShooterConstants.kLeaderMotorId);
  private final ThriftyNova followerNova = new ThriftyNova(Constants.ShooterConstants.kFollowerMotorId);

  // private final SmartMotorControllerConfig smcConfig = new
  // SmartMotorControllerConfig(this)
  // .withFollowers(Pair.of(followerNova, false))
  // .withControlMode(ControlMode.CLOSED_LOOP)
  // .withClosedLoopController(0.1, 0, 0)
  // .withFeedforward(new SimpleMotorFeedforward(0, 0.5, 0))
  // .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
  // .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
  // .withMotorInverted(false)
  // .withIdleMode(MotorMode.COAST)
  // .withStatorCurrentLimit(Amps.of(40));

  // private final SmartMotorController smc = new NovaWrapper(leaderNova,
  // DCMotor.getNEO(2), smcConfig);

  // private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smc)
  // .withDiameter(Inches.of(4))
  // .withMass(Pounds.of(1))
  // .withUpperSoftLimit(RotationsPerSecond.of(6000))
  // .withLowerSoftLimit(RotationsPerSecond.of(0))
  // .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

  // private final FlyWheel shooter = new FlyWheel(shooterConfig);

  public ShooterSubsystem() {
    SmartDashboard.putNumber("ShooterSpeed", SHOOTER_SPEED);

    leaderNova.factoryReset();
    followerNova.factoryReset();

    // leaderNova.setVoltageCompensation(12);
    // followerNova.setVoltageCompensation(12);

    leaderNova.setInverted(false);
    followerNova.setInverted(true);
  }

  // public Command setSpeed(AngularVelocity speed) {
  // return shooter.setSpeed(speed);
  // }

  public Command spinUp() {
    return run(() -> {
      leaderNova.setPercent(SHOOTER_SPEED);
      followerNova.setPercent(SHOOTER_SPEED);

      // followerNova.follow(leaderNova.getID());
      // followerNova.setPercent(0.5);
    });

    // return shooter.set(0.5);
    // return shooter.setSpeed(RotationsPerSecond.of(500));
  }

  public Command shootAtDistance(double distanceMeters) {
    return run(() -> {
      temp_distance = SHOOTING_SPEED_BY_DISTANCE.get(distanceMeters);

      leaderNova.setPercent(temp_distance);
      followerNova.setPercent(temp_distance);
    });
  }

  public Command stop() {
    return run(() -> {
      leaderNova.setPercent(0);
      followerNova.setPercent(0);
      // followerNova.setPercent(0.5);
    });
    // return shooter.set(0);
  }

  // public AngularVelocity getSpeed() {
  // return shooter.getSpeed();
  // }

  // public Command set(double dutyCycle) {
  // return shooter.set(dutyCycle);
  // }

  // public Command sysId() {
  // return shooter.sysId(Volts.of(10), Volts.of(2).per(Second), Seconds.of(10));
  // }

  @Override
  public void periodic() {
    // shooter.updateTelemetry();
    SHOOTER_SPEED = SmartDashboard.getNumber("ShooterSpeed", SHOOTER_SPEED);

    Logger.recordOutput("Shooter/Setpoint", temp_distance);
    Logger.recordOutput("Shooter/LeaderVelocity", leaderNova.getVelocity());
    Logger.recordOutput("Shooter/FollowerVelocity", followerNova.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // shooter.simIterate();
  }

  // meters, RPS
  private static final InterpolatingDoubleTreeMap SHOOTING_SPEED_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(2.63, 0.55),
      Map.entry(3.4, 0.6),
      Map.entry(4.83, 0.69));
}
