package frc.robot.subsystems.turret;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

/**
 * Real hardware implementation of TurretIO using REV SparkMax controller.
 */
public class TurretIOSparkMax implements TurretIO {

    private static final double MAX_ANGLE_DEGREES = 90.0;
    private static final double GEAR_RATIO = 40.0; // 4:1 gearbox * 10:1 pivot = 40:1
    private static final double POSITION_TOLERANCE_DEGREES = 1.0;

    private final SparkMax spark;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pid;

    private double setpointDegrees = 0.0;

    public TurretIOSparkMax() {
        spark = new SparkMax(Constants.TurretConstants.kMotorId, MotorType.kBrushless);
        encoder = spark.getEncoder();
        pid = spark.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(10)
            .voltageCompensation(12.0)
            .closedLoopRampRate(0.1)
            .openLoopRampRate(0.1);

        // Soft limits in motor rotations
        double maxMotorRotations = MAX_ANGLE_DEGREES / 360.0 * GEAR_RATIO;
        config.softLimit
            .forwardSoftLimit((float) maxMotorRotations)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit((float) -maxMotorRotations)
            .reverseSoftLimitEnabled(true);

        // Position conversion: motor rotations -> degrees
        config.encoder
            .positionConversionFactor(360.0 / GEAR_RATIO)
            .velocityConversionFactor(360.0 / GEAR_RATIO / 60.0); // RPM -> deg/s

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(15.0, 0, 0)
            .maxMotion
                .maxVelocity(2440) // deg/s converted to motor units
                .maxAcceleration(2440);

        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.positionDegrees = encoder.getPosition();
        inputs.velocityDegreesPerSec = encoder.getVelocity();
        inputs.currentAmps = spark.getOutputCurrent();
        inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
        inputs.tempCelsius = spark.getMotorTemperature();
        inputs.setpointDegrees = setpointDegrees;
        inputs.atSetpoint = Math.abs(inputs.positionDegrees - setpointDegrees) < POSITION_TOLERANCE_DEGREES;
        inputs.atPositiveLimit = inputs.positionDegrees >= MAX_ANGLE_DEGREES - 1;
        inputs.atNegativeLimit = inputs.positionDegrees <= -MAX_ANGLE_DEGREES + 1;
    }

    @Override
    public void setPosition(double angleDegrees) {
        // Clamp to soft limits
        setpointDegrees = Math.max(-MAX_ANGLE_DEGREES, Math.min(MAX_ANGLE_DEGREES, angleDegrees));
        pid.setReference(setpointDegrees, SparkMax.ControlType.kPosition);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        setpointDegrees = encoder.getPosition(); // Track current position as setpoint
        spark.set(dutyCycle);
    }

    @Override
    public void stop() {
        setpointDegrees = encoder.getPosition();
        spark.stopMotor();
    }

    @Override
    public void setVoltage(double volts) {
        setpointDegrees = encoder.getPosition();
        spark.setVoltage(volts);
    }

    @Override
    public void resetPosition() {
        encoder.setPosition(0);
        setpointDegrees = 0;
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(kP, kI, kD);
        spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
