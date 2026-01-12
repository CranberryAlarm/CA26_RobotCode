package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.thethriftybot.ThriftyNova;

import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
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
import yams.motorcontrollers.local.NovaWrapper;

public class HopperSubsystem extends SubsystemBase {

  private static final double HOPPER_SPEED = 0.5;

  // Nova motor controller with NEO motor
  private ThriftyNova hopperNova = new ThriftyNova(Constants.HopperConstants.kHopperMotorId);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("HopperMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(4))) // 4:1 gear reduction
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(20));

  private SmartMotorController smc = new NovaWrapper(hopperNova, DCMotor.getNEO(1), smcConfig);

  private final FlyWheelConfig hopperConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("Hopper", TelemetryVerbosity.HIGH);

  private FlyWheel hopper = new FlyWheel(hopperConfig);

  public HopperSubsystem() {
  }

  /**
   * Command to run the hopper forward while held.
   */
  public Command feedCommand() {
    return hopper.set(HOPPER_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Hopper.Feed");
  }

  /**
   * Command to run the hopper in reverse while held.
   */
  public Command reverseCommand() {
    return hopper.set(-HOPPER_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Hopper.Reverse");
  }

  /**
   * Command to stop the hopper.
   */
  public Command stopCommand() {
    return hopper.set(0).withName("Hopper.Stop");
  }

  @Override
  public void periodic() {
    hopper.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    hopper.simIterate();
  }
}
