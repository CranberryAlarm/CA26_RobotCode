package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.simulation.SimulatableCANSparkMax;
import frc.robot.subsystems.leds.LEDs;

public class Coral extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Coral mInstance;
  private PeriodicIO mPeriodicIO;
  public final LEDs m_leds = LEDs.getInstance();

  public static Coral getInstance() {
    if (mInstance == null) {
      mInstance = new Coral();
    }
    return mInstance;
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    REVERSE,
    INDEX,
    READY,
    SCORE
  }

  // private ThriftyNova mLeftMotor;
  // private ThriftyNova mRightMotor;
  private SimulatableCANSparkMax mLeftMotor;
  private SimulatableCANSparkMax mRightMotor;

  private LaserCan mLaserCAN;

  private Coral() {
    super("Coral");

    mPeriodicIO = new PeriodicIO();

    mLeftMotor = new SimulatableCANSparkMax(Constants.Coral.kLeftMotorId, MotorType.kBrushless);
    mRightMotor = new SimulatableCANSparkMax(Constants.Coral.kRightMotorId, MotorType.kBrushless);

    SparkMaxConfig coralConfig = new SparkMaxConfig();

    coralConfig.idleMode(IdleMode.kBrake);

    mLeftMotor.configure(
        coralConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    mRightMotor.configure(
        coralConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    mLaserCAN = new LaserCan(Constants.Coral.kLaserId);
    try {
      mLaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
      mLaserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      mLaserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  private static class PeriodicIO {
    double rpm = 0.0;
    double speed_diff = 0.0;

    int index_debounce = 0;

    LaserCan.Measurement measurement;

    IntakeState state = IntakeState.NONE;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    mPeriodicIO.measurement = mLaserCAN.getMeasurement();

    checkAutoTasks();
  }

  @Override
  public void writePeriodicOutputs() {
    mLeftMotor.set(mPeriodicIO.rpm - mPeriodicIO.speed_diff);
    mRightMotor.set(-mPeriodicIO.rpm);
  }

  @Override
  public void stop() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.state = IntakeState.NONE;
  }

  @Override
  public void outputTelemetry() {
    putNumber("RPM/target", mPeriodicIO.rpm);

    LaserCan.Measurement measurement = mPeriodicIO.measurement;
    if (measurement != null) {
      putNumber("Laser/distance", measurement.distance_mm);
      putNumber("Laser/ambient", measurement.ambient);
      putNumber("Laser/budget_ms", measurement.budget_ms);
      putNumber("Laser/status", measurement.status);

      putBoolean("Laser/hasCoral", isHoldingCoralViaLaserCAN());
    }
  }

  @Override
  public void reset() {
    stopCoral();
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public boolean isHoldingCoralViaLaserCAN() {
    return mPeriodicIO.measurement.distance_mm < 75.0;
  }

  public void setSpeed(double rpm) {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = rpm;
  }

  public void intake() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kIntakeSpeed;
    mPeriodicIO.state = IntakeState.INTAKE;

    m_leds.setColor(Color.kYellow);
  }

  public void reverse() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kReverseSpeed;
    mPeriodicIO.state = IntakeState.REVERSE;
  }

  public void index() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kIndexSpeed;
    mPeriodicIO.state = IntakeState.INDEX;

    m_leds.setColor(Color.kBlue);
  }

  public void scoreL1() {
    mPeriodicIO.speed_diff = Constants.Coral.kSpeedDifference;
    mPeriodicIO.rpm = Constants.Coral.kL1Speed;
    mPeriodicIO.state = IntakeState.SCORE;
  }

  public void scoreL24() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kL24Speed;
    mPeriodicIO.state = IntakeState.SCORE;
  }

  public void stopCoral() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.state = IntakeState.NONE;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

  private void checkAutoTasks() {
    switch (mPeriodicIO.state) {
      case INTAKE:
        if (isHoldingCoralViaLaserCAN()) {
          mPeriodicIO.index_debounce++;

          if (mPeriodicIO.index_debounce > 10) {
            mPeriodicIO.index_debounce = 0;
            index();
          }
        }
        break;
      case INDEX:
        if (!isHoldingCoralViaLaserCAN()) {
          stopCoral();

          mPeriodicIO.state = IntakeState.READY;
          m_leds.setColor(Color.kBlue);
        }
        break;
      default:
        break;
    }
  }
}
