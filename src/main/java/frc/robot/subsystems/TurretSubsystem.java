package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
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

public class TurretSubsystem extends SubsystemBase {

  // 2 Neos, 12in diameter, 25:1 gearbox, 10:1 pivot gearing reduction,
  // non-continuous (270 FOV)
  // Total reduction: 25 * 10 = 250:1
  private SparkMax leaderSpark = new SparkMax(Constants.TurretConstants.kLeaderMotorId, MotorType.kBrushless);
  private SparkMax followerSpark = new SparkMax(Constants.TurretConstants.kFollowerMotorId, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(followerSpark, false))
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

  private SmartMotorController smc = new SparkWrapper(leaderSpark, DCMotor.getNEO(2), smcConfig);

  private final PivotConfig turretConfig = new PivotConfig(smc)
      .withHardLimit(Degrees.of(-140), Degrees.of(140))
      .withStartingPosition(Degrees.of(0))
      .withMOI(0.05)
      .withTelemetry("Turret", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(new MechanismPositionConfig().withMovementPlane(Plane.XY).withRelativePosition(new Translation3d(
        Meters.of(0), Meters.of(0), Inches.of(18)
      )));

  private Pivot turret = new Pivot(turretConfig);

  public TurretSubsystem() {
  }

  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  public Command setAngleDynamic(Supplier<Angle> turretAngleSupplier) {
    return turret.setAngle(turretAngleSupplier);
  }

  public Command center() {
    return turret.setAngle(Degrees.of(0));
  }

  public Angle getAngle() {
    return turret.getAngle();
  }

  public Command set(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  public Command sysId() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  @Override
  public void periodic() {
    turret.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}
