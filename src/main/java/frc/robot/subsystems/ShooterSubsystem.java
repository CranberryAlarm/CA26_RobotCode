package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase {

  // ==================== FLYWHEEL ====================
  // 2 Neos, 4in shooter wheels, 4:1 gearbox reduction
  private SparkMax flywheelLeaderSpark = new SparkMax(Constants.ShooterConstants.kFlywheelLeaderMotorId,
      MotorType.kBrushless);
  private SparkMax flywheelFollowerSpark = new SparkMax(Constants.ShooterConstants.kFlywheelFollowerMotorId,
      MotorType.kBrushless);

  private SmartMotorControllerConfig flywheelSMCConfig = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(flywheelFollowerSpark, false))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0, 0.5, 0))
      .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController flywheelSMC = new SparkWrapper(flywheelLeaderSpark, DCMotor.getNEO(2),
      flywheelSMCConfig);

  private final FlyWheelConfig flywheelConfig = new FlyWheelConfig(flywheelSMC)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("Flywheel", TelemetryVerbosity.HIGH);

  private FlyWheel flywheel = new FlyWheel(flywheelConfig);

  // ==================== TURRET ====================
  // 2 Neos, 12in diameter, 25:1 gearbox, 10:1 pivot gearing reduction,
  // non-continuous (270 FOV)
  // Total reduction: 25 * 10 = 250:1
  private SparkMax turretLeaderSpark = new SparkMax(Constants.ShooterConstants.kTurretLeaderMotorId,
      MotorType.kBrushless);
  private SparkMax turretFollowerSpark = new SparkMax(Constants.ShooterConstants.kTurretFollowerMotorId,
      MotorType.kBrushless);

  private SmartMotorControllerConfig turretSMCConfig = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(turretFollowerSpark, false))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(100, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(180))
      .withFeedforward(new ArmFeedforward(0, 0, 0.1))
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(25, 10)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withSoftLimit(Degrees.of(-135), Degrees.of(135)) // 270 FOV, centered at 0
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

  private SmartMotorController turretSMC = new SparkWrapper(turretLeaderSpark, DCMotor.getNEO(2), turretSMCConfig);

  private final PivotConfig turretConfig = new PivotConfig(turretSMC)
      .withHardLimit(Degrees.of(-140), Degrees.of(140))
      .withStartingPosition(Degrees.of(0))
      .withMOI(0.05)
      .withTelemetry("Turret", TelemetryVerbosity.HIGH);

  private Pivot turret = new Pivot(turretConfig);

  // ==================== HOOD ====================
  // 1 Neo, 0-90 degree variability, 50:1 reduction
  private SparkMax hoodSpark = new SparkMax(Constants.ShooterConstants.kHoodMotorId, MotorType.kBrushless);

  private SmartMotorControllerConfig hoodSMCConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(100, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(90))
      .withFeedforward(new ArmFeedforward(0, 0.3, 0.1))
      .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(50)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withSoftLimit(Degrees.of(0), Degrees.of(90))
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

  private SmartMotorController hoodSMC = new SparkWrapper(hoodSpark, DCMotor.getNEO(1), hoodSMCConfig);

  private final PivotConfig hoodConfig = new PivotConfig(hoodSMC)
      .withHardLimit(Degrees.of(-5), Degrees.of(95))
      .withStartingPosition(Degrees.of(0))
      .withMOI(0.001)
      .withTelemetry("Hood", TelemetryVerbosity.HIGH);

  private Pivot hood = new Pivot(hoodConfig);

  public ShooterSubsystem() {
  }

  // ==================== FLYWHEEL COMMANDS ====================
  public Command setFlywheelSpeed(AngularVelocity speed) {
    return flywheel.setSpeed(speed);
  }

  public Command spinUpFlywheel() {
    return flywheel.setSpeed(RotationsPerSecond.of(50));
  }

  public Command stopFlywheel() {
    return flywheel.setSpeed(RotationsPerSecond.of(0));
  }

  public AngularVelocity getFlywheelSpeed() {
    return flywheel.getSpeed();
  }

  public Command flywheelSysId() {
    return flywheel.sysId(Volts.of(10), Volts.of(2).per(Second), Seconds.of(10));
  }

  // ==================== TURRET COMMANDS ====================
  public Command setTurretAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  public Command centerTurret() {
    return turret.setAngle(Degrees.of(0));
  }

  public Angle getTurretAngle() {
    return turret.getAngle();
  }

  public Command turretSysId() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  // ==================== HOOD COMMANDS ====================
  public Command setHoodAngle(Angle angle) {
    return hood.setAngle(angle);
  }

  public Command stowHood() {
    return hood.setAngle(Degrees.of(0));
  }

  public Command maxHood() {
    return hood.setAngle(Degrees.of(90));
  }

  public Angle getHoodAngle() {
    return hood.getAngle();
  }

  public Command hoodSysId() {
    return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  @Override
  public void periodic() {
    flywheel.updateTelemetry();
    turret.updateTelemetry();
    hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    flywheel.simIterate();
    turret.simIterate();
    hood.simIterate();
  }
}
