package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
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
      // Feedback Constants (PID Constants)
      .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      // Feedforward Constants
      .withFeedforward(new ArmFeedforward(0, 0, 0))
      .withSimFeedforward(new ArmFeedforward(0, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("WristMotor", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as
      // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
      // your motor.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(Constants.AlgaeConstants.kWristMotorId, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSMC = new SparkWrapper(spark, DCMotor.getNEO(1), wristSMCConfig);

  private final ArmConfig wristConfig = new ArmConfig(sparkSMC)
      // Soft limit is applied to the SmartMotorControllers PID
      .withSoftLimits(Degrees.of(-20), Degrees.of(10))
      // Hard limit is applied to the simulation.
      .withHardLimit(Degrees.of(-30), Degrees.of(40))
      // Starting position is where your arm starts
      .withStartingPosition(Degrees.of(-5))
      // Length and mass of your arm for sim.
      .withLength(Feet.of(3))
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
    return wrist.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
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
