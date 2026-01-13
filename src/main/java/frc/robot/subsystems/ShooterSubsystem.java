package frc.robot.subsystems;

import com.thethriftybot.ThriftyNova;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import yams.motorcontrollers.local.NovaWrapper;

public class ShooterSubsystem extends SubsystemBase {

  // 2 Neos, 4in shooter wheels
  private final ThriftyNova leftNova = new ThriftyNova(Constants.ShooterConstants.kLeaderMotorId);
  private final ThriftyNova rightNova = new ThriftyNova(Constants.ShooterConstants.kFollowerMotorId);

  private final SmartMotorControllerConfig leftSmcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0, 0.5, 0))
      .withTelemetry("ShooterLeftMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private final SmartMotorControllerConfig rightSmcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0, 0.5, 0))
      .withTelemetry("ShooterRightMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private final SmartMotorController leftSmc = new NovaWrapper(leftNova, DCMotor.getNEO(1), leftSmcConfig);
  private final SmartMotorController rightSmc = new NovaWrapper(rightNova, DCMotor.getNEO(1), rightSmcConfig);

  private final FlyWheelConfig leftShooterConfig = new FlyWheelConfig(leftSmc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RotationsPerSecond.of(6000))
      .withLowerSoftLimit(RotationsPerSecond.of(0))
      .withTelemetry("ShooterLeft", TelemetryVerbosity.HIGH);

  private final FlyWheelConfig rightShooterConfig = new FlyWheelConfig(rightSmc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RotationsPerSecond.of(6000))
      .withLowerSoftLimit(RotationsPerSecond.of(0))
      .withTelemetry("ShooterRight", TelemetryVerbosity.HIGH);

  private final FlyWheel leftShooter = new FlyWheel(leftShooterConfig);
  private final FlyWheel rightShooter = new FlyWheel(rightShooterConfig);

  public ShooterSubsystem() {
  }

  public Command setSpeed(AngularVelocity speed) {
    return Commands.parallel(
        leftShooter.setSpeed(speed),
        rightShooter.setSpeed(speed)
    );
  }

  public Command spinUp() {
    return Commands.parallel(
        leftShooter.setSpeed(RotationsPerSecond.of(500)),
        rightShooter.setSpeed(RotationsPerSecond.of(500))
    );
  }

  public Command stop() {
    return Commands.parallel(
        leftShooter.set(0),
        rightShooter.set(0)
    );
  }

  public AngularVelocity getSpeed() {
    // Return average speed of both flywheels
    return RotationsPerSecond.of(
        (leftShooter.getSpeed().in(RotationsPerSecond) + rightShooter.getSpeed().in(RotationsPerSecond)) / 2.0
    );
  }

  public Command set(double dutyCycle) {
    return Commands.parallel(
        leftShooter.set(dutyCycle),
        rightShooter.set(dutyCycle)
    );
  }

  public Command sysId() {
    return Commands.parallel(
        leftShooter.sysId(Volts.of(10), Volts.of(2).per(Second), Seconds.of(10)),
        rightShooter.sysId(Volts.of(10), Volts.of(2).per(Second), Seconds.of(10))
    );
  }

  @Override
  public void periodic() {
    leftShooter.updateTelemetry();
    rightShooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    leftShooter.simIterate();
    rightShooter.simIterate();
  }
}
