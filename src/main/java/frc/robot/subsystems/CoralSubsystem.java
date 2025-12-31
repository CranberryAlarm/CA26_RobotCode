package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.AngularVelocity;
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
import yams.motorcontrollers.local.SparkWrapper;
import yams.telemetry.SmartMotorControllerTelemetryConfig;

public class CoralSubsystem extends SubsystemBase {

  // Vendor motor controller object
  private SparkMax leftSpark = new SparkMax(Constants.CoralConstants.kLeftIndexMotorId, MotorType.kBrushless);
  private SparkMax rightSpark = new SparkMax(Constants.CoralConstants.kRightIndexMotorId, MotorType.kBrushless);

  private SmartMotorControllerConfig coralSMCConfig = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(rightSpark, true))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.144, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.122, 0.485, 0))
      .withTelemetry("CoralMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController leftSparkSMC = new SparkWrapper(leftSpark, DCMotor.getNEO(1), coralSMCConfig);

  private final FlyWheelConfig coralConfig = new FlyWheelConfig(leftSparkSMC)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withUpperSoftLimit(RPM.of(10000))
      .withLowerSoftLimit(RPM.of(-10000))
      .withTelemetry("CoralIndexer", TelemetryVerbosity.HIGH);

  private FlyWheel coral = new FlyWheel(coralConfig);

  private final LaserCan mLaserCAN = new LaserCan(0);

  public CoralSubsystem() {
    try {
      mLaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
      mLaserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      mLaserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  public AngularVelocity getVelocity() {
    return coral.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return coral.setSpeed(speed);
  }

  public Command set(double dutyCycle) {
    return coral.set(dutyCycle);
  }

  public Command sysId() {
    return coral.sysId(Volts.of(10), Volts.of(2).per(Second), Seconds.of(10));
  }

  @Override
  public void periodic() {
    coral.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    coral.simIterate();
  }
}
