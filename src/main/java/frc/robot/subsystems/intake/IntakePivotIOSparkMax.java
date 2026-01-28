package frc.robot.subsystems.intake;

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
 * Real hardware implementation of IntakePivotIO using SparkMax.
 */
public class IntakePivotIOSparkMax implements IntakePivotIO {

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 150.0;
    private static final double POSITION_TOLERANCE_DEGREES = 5.0;
    private static final double GEAR_RATIO = 5.0 * 5.0 * (60.0 / 18.0); // ~83.33:1

    private final SparkMax spark;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pid;

    private double setpointDegrees = 0.0;

    public IntakePivotIOSparkMax() {
        spark = new SparkMax(Constants.IntakeConstants.kPivotMotorId, MotorType.kBrushless);
        encoder = spark.getEncoder();
        pid = spark.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(10)
            .voltageCompensation(12.0)
            .closedLoopRampRate(0.1)
            .openLoopRampRate(0.1);

        // Soft limits
        double maxMotorRotations = MAX_ANGLE_DEGREES / 360.0 * GEAR_RATIO;
        config.softLimit
            .forwardSoftLimit((float) maxMotorRotations)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(0)
            .reverseSoftLimitEnabled(true);

        // Position conversion
        config.encoder
            .positionConversionFactor(360.0 / GEAR_RATIO)
            .velocityConversionFactor(360.0 / GEAR_RATIO / 60.0);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(25, 0, 0)
            .maxMotion
                .maxVelocity(360)
                .maxAcceleration(360);

        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        inputs.positionDegrees = encoder.getPosition();
        inputs.velocityDegreesPerSec = encoder.getVelocity();
        inputs.currentAmps = spark.getOutputCurrent();
        inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
        inputs.tempCelsius = spark.getMotorTemperature();
        inputs.setpointDegrees = setpointDegrees;
        inputs.atSetpoint = Math.abs(inputs.positionDegrees - setpointDegrees) < POSITION_TOLERANCE_DEGREES;
    }

    @Override
    public void setPosition(double angleDegrees) {
        setpointDegrees = Math.max(MIN_ANGLE_DEGREES, Math.min(MAX_ANGLE_DEGREES, angleDegrees));
        pid.setReference(setpointDegrees, SparkMax.ControlType.kPosition);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        setpointDegrees = encoder.getPosition();
        spark.set(dutyCycle);
    }

    @Override
    public void stop() {
        setpointDegrees = encoder.getPosition();
        spark.stopMotor();
    }

    @Override
    public void resetPosition() {
        encoder.setPosition(0);
        setpointDegrees = 0;
    }
}
