package frc.robot.subsystems;

import com.thethriftybot.ThriftyNova;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;

public class IntakeSubsystem extends SubsystemBase {

  private static final double INTAKE_SPEED = 0.5;

  // ThriftyNova controlling the intake roller
  private ThriftyNova rollerNova = new ThriftyNova(Constants.IntakeConstants.kRollerMotorId);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) // Direct drive, adjust if geared
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController smc = new NovaWrapper(rollerNova, DCMotor.getNeoVortex(1), smcConfig);

  private final FlyWheelConfig intakeConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH);

  private FlyWheel intake = new FlyWheel(intakeConfig);

  // 5:1, 5:1, 60/18 reduction
  private SmartMotorControllerConfig intakePivotSmartMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(10, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(360))
      .withFeedforward(new ArmFeedforward(0, 0, 0.0))
      .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0 / 18.0, 42)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(10))
      .withClosedLoopRampRate(Seconds.of(0.1));

  private ThriftyNova pivotMotor = new ThriftyNova(Constants.IntakeConstants.kPivotMotorId);

  private SmartMotorController intakePivotController = new NovaWrapper(pivotMotor, DCMotor.getNeoVortex(1),
      intakePivotSmartMotorConfig);

  private final ArmConfig intakePivotConfig = new ArmConfig(intakePivotController)
      // .withSoftLimits(Degrees.of(0), Degrees.of(185))
      // .withHardLimit(Degrees.of(0), Degrees.of(190))
      .withStartingPosition(Degrees.of(0))
      .withLength(Feet.of(1))
      .withMass(Pounds.of(2)) // Reis says: 2 pounds, not a lot
      .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

  private Arm intakePivot = new Arm(intakePivotConfig);

  public IntakeSubsystem() {
    pivotMotor.factoryReset();
  }

  /**
   * Command to run the intake while held.
   */
  public Command intakeCommand() {
    return intake.set(INTAKE_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.Run");
  }

  /**
   * Command to eject while held.
   */
  public Command ejectCommand() {
    return intake.set(-INTAKE_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.Eject");
  }

  public Command setPivotAngle(Angle angle) {
    return intakePivot.setAngle(angle).withName("IntakePivot.SetAngle");
  }

  public Command rezero() {
    return Commands.runOnce(() -> pivotMotor.setEncoderPosition(0), this).withName("IntakePivot.Rezero");
  }

  @Override
  public void periodic() {
    intake.updateTelemetry();
    intakePivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
    intakePivot.simIterate();
  }
}
