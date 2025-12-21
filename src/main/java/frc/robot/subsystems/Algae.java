package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.simulation.SimulatableCANSparkMax;
import frc.robot.wrappers.REVThroughBoreEncoder;

public class Algae extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Algae mInstance;
  private PeriodicIO mPeriodicIO;

  public static Algae getInstance() {
    if (mInstance == null) {
      mInstance = new Algae();
    }
    return mInstance;
  }

  public enum IntakeState {
    NONE,
    STOW,
    DEALGAE,
    GROUND
  }

  private SimulatableCANSparkMax mWristMotor;
  private final ProfiledPIDController mWristPIDController;
  private final ArmFeedforward mWristFeedForward;

  private SimulatableCANSparkMax mIntakeMotor;

  private final REVThroughBoreEncoder mWristAbsEncoder = new REVThroughBoreEncoder(Constants.Algae.kWristEncoderId);

  private Algae() {
    super("Algae");

    mPeriodicIO = new PeriodicIO();

    // WRIST
    mWristMotor = new SimulatableCANSparkMax(Constants.Algae.kWristMotorId, MotorType.kBrushless);
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    wristConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(Constants.Algae.kMaxWristCurrent)
        .inverted(true);

    mWristMotor.configure(
        wristConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Wrist PID
    mWristPIDController = new ProfiledPIDController(
        Constants.Algae.kWristP,
        Constants.Algae.kWristI,
        Constants.Algae.kWristD,
        new TrapezoidProfile.Constraints(
            Constants.Algae.kWristMaxVelocity,
            Constants.Algae.kWristMaxAcceleration));

    // Wrist Feedforward
    mWristFeedForward = new ArmFeedforward(
        Constants.Algae.kWristKS,
        Constants.Algae.kWristKG,
        Constants.Algae.kWristKV,
        Constants.Algae.kWristKA);

    // INTAKE
    mIntakeMotor = new SimulatableCANSparkMax(Constants.Algae.kIntakeMotorId, MotorType.kBrushless);
    SparkMaxConfig intakeConfig = new SparkMaxConfig();

    intakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.Algae.kMaxIntakeCurrent)
        .inverted(true);

    mIntakeMotor.configure(
        intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private static class PeriodicIO {
    double wrist_target_angle = 0.0;
    double wrist_voltage = 0.0;

    double intake_power = 0.0;

    IntakeState state = IntakeState.STOW;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    double pidCalc = mWristPIDController.calculate(getWristAngle(), mPeriodicIO.wrist_target_angle);
    double ffCalc = mWristFeedForward.calculate(Math.toRadians(getWristReferenceToHorizontal()),
        Math.toRadians(mWristPIDController.getSetpoint().velocity));

    mPeriodicIO.wrist_voltage = pidCalc + ffCalc;
  }

  @Override
  public void writePeriodicOutputs() {
    mWristMotor.set(mPeriodicIO.wrist_voltage);

    mIntakeMotor.set(mPeriodicIO.intake_power);
  }

  @Override
  public void stop() {
    mPeriodicIO.wrist_voltage = 0.0;
    mPeriodicIO.wrist_target_angle = Constants.Algae.kStowAngle;

    mWristMotor.set(0.0);
    mIntakeMotor.set(0.0);
  }

  @Override
  public void outputTelemetry() {
    putNumber("Wrist/Position", getWristAngle());
    putNumber("Wrist/Target", mPeriodicIO.wrist_target_angle);
    putNumber("Wrist/Current", mWristMotor.getOutputCurrent());
    putNumber("Wrist/Output", mWristMotor.getAppliedOutput());
    putNumber("Wrist/Voltage", mPeriodicIO.wrist_voltage);
    putNumber("Wrist/Frequency", mWristAbsEncoder.getFrequency());

    putNumber("Intake/Current", mIntakeMotor.getOutputCurrent());
    putNumber("Intake/Output", mIntakeMotor.getAppliedOutput());
    putNumber("Intake/Power", mPeriodicIO.intake_power);
  }

  @Override
  public void reset() {
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void stow() {
    mPeriodicIO.wrist_target_angle = Constants.Algae.kStowAngle;

    mPeriodicIO.state = IntakeState.STOW;
    // mPeriodicIO.intake_power = 0.0;
  }

  public void grabAlgae() {
    mPeriodicIO.wrist_target_angle = Constants.Algae.kDeAlgaeAngle;
    mPeriodicIO.intake_power = Constants.Algae.kIntakeSpeed;

    mPeriodicIO.state = IntakeState.DEALGAE;
  }

  public void score() {
    if (mPeriodicIO.state == IntakeState.GROUND) {
      mPeriodicIO.intake_power = -Constants.Algae.kEjectSpeed;
    } else {
      mPeriodicIO.intake_power = Constants.Algae.kEjectSpeed;
    }
  }

  public void groundIntake() {
    mPeriodicIO.wrist_target_angle = Constants.Algae.kGroundIntakeAngle;
    mPeriodicIO.intake_power = Constants.Algae.kGroundIntakeSpeed;

    mPeriodicIO.state = IntakeState.GROUND;
  }

  public void stopAlgae() {
    mPeriodicIO.intake_power = 0.0;
    mPeriodicIO.wrist_target_angle = Constants.Algae.kStowAngle;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

  public double getWristAngle() {
    // TODO:
    // This used to be `getAbsolutePosition` in the old API
    // but I'm not sure if `get` is the correct replacement
    return Units.rotationsToDegrees(mWristAbsEncoder.get());
  }

  public double getWristReferenceToHorizontal() {
    return getWristAngle() - Constants.Algae.kWristOffset;
  }

  public IntakeState getState() {
    return mPeriodicIO.state;
  }

  // public double getSpeedFromState(IntakeState state) {
  // switch (state) {
  // case NONE:
  // return 0.0;
  // case INTAKE:
  // return RobotConstants.config.Intake.k_intakeSpeed;
  // case INDEX:
  // return RobotConstants.config.Intake.k_ejectSpeed;
  // case READY:
  // return RobotConstants.config.Intake.k_feedShooterSpeed;
  // default:
  // return 0.0;
  // }
  // }
}
