package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import frc.robot.Constants;
import frc.robot.Robot;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Turret IO using YAMS for motor control and simulation.
 * Works for both real hardware and simulation - YAMS handles both internally.
 */
public class TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    public double positionDegrees = 0.0;
    public double velocityDegreesPerSec = 0.0;
    public double currentAmps = 0.0;
    public double appliedVolts = 0.0;
    public double tempCelsius = 0.0;
    public double setpointDegrees = 0.0;
    public boolean atSetpoint = false;
    public boolean atPositiveLimit = false;
    public boolean atNegativeLimit = false;
  }

  private static final double MAX_ANGLE_DEGREES = 90.0;
  private static final double POSITION_TOLERANCE_DEGREES = 1.0;
  private static final Translation3d TURRET_TRANSLATION = new Translation3d(-0.205, 0.0, 0.375);

  private final SparkMax spark;
  private final SmartMotorController smc;
  private final Pivot pivot;

  private double setpointDegrees = 0.0;

  public TurretIO() {
    spark = new SparkMax(Constants.TurretConstants.kMotorId, MotorType.kBrushless);

    // Configure YAMS SmartMotorController with all PID/FF settings
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(null) // No subsystem - IO handles it
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(15.0, 0, 0, DegreesPerSecond.of(2440), DegreesPerSecondPerSecond.of(2440))
        .withFeedforward(new SimpleMotorFeedforward(0, 7.5, 0))
        .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(4, 10)))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withSoftLimit(Degrees.of(-MAX_ANGLE_DEGREES), Degrees.of(MAX_ANGLE_DEGREES))
        .withStatorCurrentLimit(Amps.of(10))
        .withClosedLoopRampRate(Seconds.of(0.1))
        .withOpenLoopRampRate(Seconds.of(0.1));

    smc = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

    // Configure YAMS Pivot mechanism
    PivotConfig pivotConfig = new PivotConfig(smc)
        .withHardLimit(Degrees.of(-MAX_ANGLE_DEGREES - 5), Degrees.of(MAX_ANGLE_DEGREES + 5))
        .withStartingPosition(Degrees.of(0))
        .withMOI(0.05)
        .withTelemetry("Turret", TelemetryVerbosity.LOW) // Low since AK handles main telemetry
        .withMechanismPositionConfig(
            new MechanismPositionConfig().withMovementPlane(Plane.XY).withRelativePosition(TURRET_TRANSLATION));

    pivot = new Pivot(pivotConfig);
  }

  public void updateInputs(TurretIOInputs inputs) {
    // Run YAMS simulation if in sim mode
    if (!Robot.isReal()) {
      pivot.simIterate();
    }

    // Update telemetry from YAMS
    pivot.updateTelemetry();

    // Read values from YAMS
    inputs.positionDegrees = pivot.getAngle().in(Degrees);
    inputs.velocityDegreesPerSec = spark.getEncoder().getVelocity(); // Already converted by encoder config
    inputs.currentAmps = spark.getOutputCurrent();
    inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.tempCelsius = spark.getMotorTemperature();
    inputs.setpointDegrees = setpointDegrees;
    inputs.atSetpoint = Math.abs(inputs.positionDegrees - setpointDegrees) < POSITION_TOLERANCE_DEGREES;
    inputs.atPositiveLimit = inputs.positionDegrees >= MAX_ANGLE_DEGREES - 1;
    inputs.atNegativeLimit = inputs.positionDegrees <= -MAX_ANGLE_DEGREES + 1;
  }

  public void setPosition(double angleDegrees) {
    setpointDegrees = Math.max(-MAX_ANGLE_DEGREES, Math.min(MAX_ANGLE_DEGREES, angleDegrees));
    smc.setPosition(Degrees.of(setpointDegrees));
  }

  public void setDutyCycle(double dutyCycle) {
    setpointDegrees = pivot.getAngle().in(Degrees);
    smc.setDutyCycle(dutyCycle);
  }

  public void stop() {
    setpointDegrees = pivot.getAngle().in(Degrees);
    smc.setDutyCycle(0);
  }

  public void setVoltage(double volts) {
    setpointDegrees = pivot.getAngle().in(Degrees);
    smc.setVoltage(Volts.of(volts));
  }

  public void resetPosition() {
    spark.getEncoder().setPosition(0);
    setpointDegrees = 0;
  }

  /**
   * Gets the underlying Pivot for direct access if needed.
   */
  public Pivot getPivot() {
    return pivot;
  }
}
