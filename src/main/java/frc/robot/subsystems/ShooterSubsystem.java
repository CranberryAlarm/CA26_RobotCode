package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase {
  private double temp_distance = 0.0;

  // 2 Neos, 4in shooter wheels
  // private final ThriftyNova leaderNova = new ThriftyNova(
  // Constants.ShooterConstants.kLeaderMotorId,
  // ThriftyNova.MotorType.NEO);

  // private final ThriftyNova followerNova = new ThriftyNova(
  // Constants.ShooterConstants.kFollowerMotorId,
  // ThriftyNova.MotorType.NEO);

  private final SparkMax leaderSpark = new SparkMax(Constants.ShooterConstants.kLeaderMotorId,
      MotorType.kBrushless);

  private final SparkMax followerSpark = new SparkMax(Constants.ShooterConstants.kFollowerMotorId,
      MotorType.kBrushless);

  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(followerSpark, true))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.00936, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private final SmartMotorController smc = new SparkWrapper(leaderSpark, DCMotor.getNEO(2), smcConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(0))
      .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

  private final FlyWheel shooter = new FlyWheel(shooterConfig);

  public ShooterSubsystem() {
    // leaderNova.factoryReset();
    // followerNova.factoryReset();

    // leaderNova.setVoltageCompensation(12);
    // followerNova.setVoltageCompensation(12);

    // leaderNova.setInverted(false);
    // followerNova.setInverted(true);

    // followerNova
    // .setInversion(true)
    // .follow(leaderNova.getID());
  }

  public Command setSpeed(AngularVelocity speed) {
    return shooter.setSpeed(speed);
  }

  public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier) {
    return shooter.setSpeed(speedSupplier.get());
  }

  public Command spinUp() {
    return setSpeed(RPM.of(5500));

    // return setSpeed(RotationsPerSecond.of(50));

    // return run(() -> {
    // // followerNova.follow(leaderNova.getID());
    // // followerNova.setInverted(true);

    // // leaderNova.setPercent(SHOOTER_SPEED);
    // // followerNova.setPercent(SHOOTER_SPEED);

    // // followerNova.setPercent(0.5);
    // });

    // return shooter.set(0.5);
    // return shooter.setSpeed(RotationsPerSecond.of(500));
  }

  public Command shootAtDistance(double distanceMeters) {
    return run(() -> {
      temp_distance = SHOOTING_SPEED_BY_DISTANCE.get(distanceMeters);

      // leaderNova.setPercent(temp_distance);
      // followerNova.setPercent(temp_distance);
    });
  }

  public Command stop() {
    return setSpeed(RPM.of(0));
    // return run(() -> {

    // // leaderNova.setPercent(0);
    // // followerNova.setPercent(0);
    // // followerNova.setPercent(0.5);
    // });
    // return shooter.set(0);
  }

  public AngularVelocity getSpeed() {
    return shooter.getSpeed();
  }

  // public Command set(double dutyCycle) {
  // return shooter.set(dutyCycle);
  // }

  public Command sysId() {
    return shooter.sysId(Volts.of(12), Volts.of(3).per(Second), Seconds.of(7));
  }

  @Override
  public void periodic() {
    // shooter.updateTelemetry();
    // SHOOTER_SPEED = SmartDashboard.getNumber("ShooterSpeed", SHOOTER_SPEED);

    Logger.recordOutput("Shooter/Setpoint", temp_distance);
    Logger.recordOutput("Shooter/LeaderVelocity", leaderSpark.getEncoder().getVelocity());
    Logger.recordOutput("Shooter/FollowerVelocity", followerSpark.getEncoder().getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // shooter.simIterate();
  }

  private Distance wheelRadius() {
    return Inches.of(4).div(2);
  }

  public LinearVelocity getTangentialVelocity() {
    // Calculate tangential velocity at the edge of the wheel and convert to
    // LinearVelocity

    return MetersPerSecond.of(getSpeed().in(RadiansPerSecond)
        * wheelRadius().in(Meters));
  }

  // meters, RPS
  private static final InterpolatingDoubleTreeMap SHOOTING_SPEED_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(2.63, 0.55),
      Map.entry(3.4, 0.6),
      Map.entry(4.83, 0.69));
}
