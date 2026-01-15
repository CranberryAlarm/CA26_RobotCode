package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class HoodSubsystem extends SubsystemBase {

  // 1 Neo, 0-90 degree variability, 50:1 reduction
  // private SparkMax spark = new SparkMax(Constants.HoodConstants.kMotorId,
  // MotorType.kBrushless);

  // private SmartMotorControllerConfig smcConfig = new
  // SmartMotorControllerConfig(this)
  // .withControlMode(ControlMode.CLOSED_LOOP)
  // .withClosedLoopController(100, 0, 0, DegreesPerSecond.of(90),
  // DegreesPerSecondPerSecond.of(90))
  // .withFeedforward(new ArmFeedforward(0, 0.3, 0.1))
  // .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
  // .withGearing(new MechanismGearing(GearBox.fromReductionStages(50)))
  // .withMotorInverted(false)
  // .withIdleMode(MotorMode.BRAKE)
  // .withSoftLimit(Degrees.of(0), Degrees.of(90))
  // .withStatorCurrentLimit(Amps.of(40))
  // .withClosedLoopRampRate(Seconds.of(0.1))
  // .withOpenLoopRampRate(Seconds.of(0.1));

  // private SmartMotorController smc = new SparkWrapper(spark, DCMotor.getNEO(1),
  // smcConfig);

  // private PivotConfig hoodConfig = new PivotConfig(smc)
  // .withHardLimit(Degrees.of(-5), Degrees.of(95))
  // .withStartingPosition(Degrees.of(0))
  // .withMOI(0.001)
  // .withTelemetry("Hood", TelemetryVerbosity.HIGH);

  // private Pivot hood = new Pivot(hoodConfig);

  public HoodSubsystem() {
  }

  public Command setAngle(Angle angle) {
    // return hood.setAngle(angle);
    return Commands.runOnce(() -> {
    });
  }

  public Command setAngleDynamic(Supplier<Angle> hoodAngleSupplier) {
    // return hood.setAngle(hoodAngleSupplier);
    return Commands.runOnce(() -> {
    });
  }

  public Command stow() {
    return setAngle(Degrees.of(0));
  }

  public Command max() {
    return setAngle(Degrees.of(90));
  }

  public Angle getAngle() {
    return Degrees.of(45);
    // return hood.getAngle();
  }

  public Command set(double dutyCycle) {
    return Commands.runOnce(() -> {
    });
  }

  public Command sysId() {
    // return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
    return Commands.runOnce(() -> {
    });
  }

  @Override
  public void periodic() {
    // hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // hood.simIterate();
  }
}
