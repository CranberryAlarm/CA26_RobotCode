package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

/**
 * Real hardware implementation of ShooterIO using REV SparkMax controllers.
 */
public class ShooterIOSparkMax implements ShooterIO {

  private final SparkMax leaderSpark;
  private final SparkMax followerSpark;
  private final RelativeEncoder leaderEncoder;
  private final RelativeEncoder followerEncoder;
  private final SparkClosedLoopController leaderPID;

  // Feedforward constants
  private static final double kS = 0.191; // Static friction voltage
  private static final double kV = 0.11858; // Velocity feedforward (V per RPM)

  private double velocitySetpointRPM = 0.0;

  public ShooterIOSparkMax() {
    leaderSpark = new SparkMax(Constants.ShooterConstants.kLeaderMotorId, MotorType.kBrushless);
    followerSpark = new SparkMax(Constants.ShooterConstants.kFollowerMotorId, MotorType.kBrushless);

    leaderEncoder = leaderSpark.getEncoder();
    followerEncoder = followerSpark.getEncoder();
    leaderPID = leaderSpark.getClosedLoopController();

    // Configure leader
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0);
    leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.00936, 0, 0);
    leaderConfig.encoder
        .positionConversionFactor(1.0) // 1:1 gearing
        .velocityConversionFactor(1.0);

    leaderSpark.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure follower
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
        .inverted(true) // Inverted relative to leader
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0)
        .follow(leaderSpark, true);

    followerSpark.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leaderVelocityRPM = leaderEncoder.getVelocity();
    inputs.followerVelocityRPM = followerEncoder.getVelocity();
    inputs.leaderCurrentAmps = leaderSpark.getOutputCurrent();
    inputs.followerCurrentAmps = followerSpark.getOutputCurrent();
    inputs.leaderAppliedVolts = leaderSpark.getAppliedOutput() * leaderSpark.getBusVoltage();
    inputs.leaderTempCelsius = leaderSpark.getMotorTemperature();
    inputs.followerTempCelsius = followerSpark.getMotorTemperature();
    inputs.velocitySetpointRPM = velocitySetpointRPM;
    inputs.atSetpoint = Math.abs(inputs.leaderVelocityRPM - velocitySetpointRPM) < 100; // 100 RPM tolerance
  }

  @Override
  public void setVelocity(double velocityRPM) {
    velocitySetpointRPM = velocityRPM;

    if (velocityRPM == 0) {
      stop();
      return;
    }

    // Calculate feedforward
    double ffVolts = kS * Math.signum(velocityRPM) + kV * velocityRPM;

    // Use closed-loop velocity control with feedforward
    leaderPID.setReference(
        velocityRPM,
        SparkMax.ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    velocitySetpointRPM = 0;
    leaderSpark.set(dutyCycle);
  }

  @Override
  public void stop() {
    velocitySetpointRPM = 0;
    leaderSpark.stopMotor();
  }

  @Override
  public void setVoltage(double volts) {
    velocitySetpointRPM = 0;
    leaderSpark.setVoltage(volts);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(kP, kI, kD);
    leaderSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
