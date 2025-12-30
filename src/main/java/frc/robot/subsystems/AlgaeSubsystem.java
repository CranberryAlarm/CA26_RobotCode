package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class AlgaeSubsystem extends SubsystemBase {
  private SmartMotorControllerConfig wristSMCConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(200, 0, 0, DegreesPerSecond.of(1080), DegreesPerSecondPerSecond.of(1080))
      .withFeedforward(new ArmFeedforward(0, 0, 0.1))
      .withTelemetry("WristMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(125, 48.0/36.0)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(Constants.AlgaeConstants.kWristMotorId, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSMC = new SparkWrapper(spark, DCMotor.getNEO(1), wristSMCConfig);

  private final ArmConfig wristConfig = new ArmConfig(sparkSMC)
      // Soft limit is applied to the SmartMotorControllers PID
      .withSoftLimits(Degrees.of(-135), Degrees.of(5))
      // Hard limit is applied to the simulation.
      .withHardLimit(Degrees.of(-140), Degrees.of(10))
      // Starting position is where your arm starts
      .withStartingPosition(Degrees.of(0))
      // Length and mass of your arm for sim.
      .withLength(Feet.of(1.5))
      .withMass(Pounds.of(1))
      // Telemetry name and verbosity for the arm.
      .withTelemetry("Wrist", TelemetryVerbosity.HIGH);

  // Wrist Mechanism
  private Arm wrist = new Arm(wristConfig);

  public AlgaeSubsystem() {
  }

  public Command setAngle(Angle angle) {
    return wrist.setAngle(angle);
  }

  public Command set(double dutycycle) {
    return wrist.set(dutycycle);
  }

  public Command sysId() {
    return wrist.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  @Override
  public void periodic() {
    wrist.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    wrist.simIterate();
  }
}
