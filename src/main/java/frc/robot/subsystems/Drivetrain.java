package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.measure.Current;
import edu.wpi.first.units.measure.measure.Distance;
import edu.wpi.first.units.measure.measure.LinearVelocity;
import edu.wpi.first.units.measure.measure.Mass;
import edu.wpi.first.units.measure.measure.MomentOfInertia;
import edu.wpi.first.units.measure.measure.MutDistance;
import edu.wpi.first.units.measure.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.measure.MutVoltage;
import edu.wpi.first.units.measure.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.simulation.Field;
import frc.robot.simulation.SimulatableCANSparkMax;

public class Drivetrain extends Subsystem {
  // 1 meters per second.
  public static final double kMaxSpeed = 1.0;
  public static final double kMaxBoostSpeed = 2.0;

  // 3 meters per second.
  public static final double kMaxAcceleration = 8.0;

  // 0.7 rotations per second.
  public static final double kMaxAngularSpeed = Math.PI * 1.0;

  private static final double kSlowModeRotScale = 0.1;
  private static final double kSpeedModeScale = 2.0;
  private static final double kTippyModeScale = 0.7;

  private static final double kTrackWidth = Units.inchesToMeters(20.75);
  private static final double kWheelRadius = Units.inchesToMeters(3.0);
  private static final double kGearRatio = 10.71;
  private static final double kMetersPerRev = (2.0 * Math.PI * kWheelRadius) / kGearRatio;

  private final SimulatableCANSparkMax mLeftLeader = new SimulatableCANSparkMax(Constants.Drive.kFLMotorId,
      MotorType.kBrushless);
  private final SimulatableCANSparkMax mLeftFollower = new SimulatableCANSparkMax(Constants.Drive.kBLMotorId,
      MotorType.kBrushless);
  private final SimulatableCANSparkMax mRightLeader = new SimulatableCANSparkMax(Constants.Drive.kFRMotorId,
      MotorType.kBrushless);
  private final SimulatableCANSparkMax mRightFollower = new SimulatableCANSparkMax(Constants.Drive.kBRMotorId,
      MotorType.kBrushless);

  private final RelativeEncoder mLeftEncoder;
  private final RelativeEncoder mRightEncoder;

  private final PIDController mLeftPIDController = new PIDController(Constants.Drive.kP, Constants.Drive.kI,
      Constants.Drive.kD);
  private final PIDController mRightPIDController = new PIDController(Constants.Drive.kP, Constants.Drive.kI,
      Constants.Drive.kD);

  private final AHRS mGyro = new AHRS();
  private final Elevator mElevator = Elevator.getInstance();

  private final DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry mOdometry;
  private boolean mPoseWasSet = false;

  private final SimpleMotorFeedforward mLeftFeedforward = new SimpleMotorFeedforward(Constants.Drive.kS,
      Constants.Drive.kV, Constants.Drive.kA);
  private final SimpleMotorFeedforward mRightFeedforward = new SimpleMotorFeedforward(Constants.Drive.kS,
      Constants.Drive.kV, Constants.Drive.kA);

  private final ModuleConfig moduleConfig = new ModuleConfig(
      Distance.ofBaseUnits(kWheelRadius, Meters), // wheel radius
      LinearVelocity.ofBaseUnits(kMaxSpeed, MetersPerSecond),
      1.0, // coefficient of friction (1.0 is a placeholder value)
      DCMotor.getCIM(2),
      Current.ofBaseUnits(1, Amps), // Another placeholder
      2);

  private final RobotConfig robotConfig = new RobotConfig(
      Mass.ofBaseUnits(30, Kilogram),
      MomentOfInertia.ofBaseUnits(1, KilogramSquareMeters),
      moduleConfig,
      Distance.ofBaseUnits(mKinematics.trackWidthMeters, Meters));

  /*********
   * SysId *
   *********/

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.of(0).mutableCopy();
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutDistance m_distance = Meters.of(0).mutableCopy();
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.of(0).mutableCopy();

  private final SysIdRoutine mSysIdRoutine;

  // Simulation classes help us simulate our robot
  private final Field m_field = Field.getInstance();
  private final LinearSystem<N2, N2, N2> mDrivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5,
      0.3);
  private final DifferentialDrivetrainSim mDrivetrainSimulator = new DifferentialDrivetrainSim(
      mDrivetrainSystem, DCMotor.getCIM(2), kGearRatio, kTrackWidth, kWheelRadius, null);

  private static Drivetrain mInstance;
  private static PeriodicIO mPeriodicIO;

  public static Drivetrain getInstance() {
    if (mInstance == null) {
      mInstance = new Drivetrain();
    }
    return mInstance;
  }

  public Drivetrain() {
    super("Drivetrain");

    mGyro.reset();

    var globalConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kCoast);

    var encoderConfig = new SparkMaxConfig().encoder
        // The "native units" for the SparkMax is motor rotations:
        // Conversion factor = (distance traveled per motor shaft rotation)
        .positionConversionFactor(kMetersPerRev)

        // The "native units" for the SparkMax is RPM:
        // Conversion factor = (distance traveled per motor shaft rotation) / (60
        // seconds)
        .velocityConversionFactor(kMetersPerRev / 60);

    var leftLeaderConfig = new SparkMaxConfig()
        .apply(globalConfig)
        .apply(encoderConfig);

    var leftFollowerConfig = new SparkMaxConfig()
        .apply(globalConfig)
        .follow(mLeftLeader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    var rightLeaderConfig = new SparkMaxConfig()
        .apply(globalConfig)
        .apply(encoderConfig)
        .inverted(true);

    var rightFollowerConfig = new SparkMaxConfig()
        .apply(globalConfig)
        .apply(rightLeaderConfig)
        .follow(mRightLeader);

    mLeftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mLeftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mRightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mRightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mLeftEncoder = mLeftLeader.getEncoder();
    mRightEncoder = mRightLeader.getEncoder();

    mLeftEncoder.setPosition(0.0);
    mRightEncoder.setPosition(0.0);

    mOdometry = new DifferentialDriveOdometry(mGyro.getRotation2d(),
        mLeftEncoder.getPosition(), mRightEncoder.getPosition());

    mPeriodicIO = new PeriodicIO();

    mSysIdRoutine = new SysIdRoutine(
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            (Voltage volts) -> {
              // System.out.println("OPE:" + volts);
              mLeftLeader.setVoltage(volts.in(Volts));
              mRightLeader.setVoltage(volts.in(Volts));
            },
            // Tell SysId how to record a frame of data for each motor on the mechanism
            // being characterized.
            log -> {
              // Record a frame for the left motors. Since these share an encoder, we consider
              // the entire group to be one motor.
              log.motor("drive-left")
                  .voltage(m_appliedVoltage.mut_replace(
                      mLeftLeader.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(mLeftEncoder.getPosition(), Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(mLeftEncoder.getVelocity(), MetersPerSecond));
              // Record a frame for the right motors. Since these share an encoder, we
              // consider the entire group to be one motor.
              log.motor("drive-right")
                  .voltage(m_appliedVoltage.mut_replace(
                      mRightLeader.getAppliedOutput() * RobotController
                          .getBatteryVoltage(),
                      Volts))
                  .linearPosition(m_distance.mut_replace(mRightEncoder.getPosition(), Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(mRightEncoder.getVelocity(), MetersPerSecond));
            },
            // Tell SysId to make generated commands require this subsystem, suffix test
            // state in WPILog with this subsystem's name ("drive")
            this));

  }

  private static class PeriodicIO {
    DifferentialDriveWheelSpeeds diffWheelSpeeds = new DifferentialDriveWheelSpeeds(0.0, 0.0);
    boolean slowMode = false;
    boolean speedMode = false;
    double leftVoltage = 0.0;
    double rightVoltage = 0.0;
  }

  /**
   * Gets whether the bot pose has previously been set
   */
  public boolean poseWasSet() {
    return mPoseWasSet;
  }

  /**
   * Sets whether slow mode should be used
   *
   * @param slowMode Should slow mode be used
   */
  public void slowMode(boolean slowMode) {
    mPeriodicIO.slowMode = slowMode;
  }

  /**
   * Sets whether speed mode should be used
   *
   * @param speedMode Should speed mode be used
   */
  public void speedMode(boolean speedMode) {
    mPeriodicIO.speedMode = speedMode;
  }

  /**
   * Controls the robot using arcade drive.
   *
   * @param xSpeed the speed for the x axis
   * @param rot    the rotation
   */
  public void drive(double xSpeed, double rot) {
    if (mPeriodicIO.slowMode) {
      mPeriodicIO.diffWheelSpeeds = mKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot * kSlowModeRotScale));
    } else if (mPeriodicIO.speedMode) {
      mPeriodicIO.diffWheelSpeeds = mKinematics
          .toWheelSpeeds(new ChassisSpeeds(xSpeed * kSpeedModeScale, 0, rot * kSlowModeRotScale));
    } else {
      Elevator.ElevatorState state = mElevator.getState();

      boolean highSetPoint = state == Elevator.ElevatorState.L4 ||
          state == Elevator.ElevatorState.L3 ||
          state == Elevator.ElevatorState.A2;

      double scale = (highSetPoint ? kTippyModeScale : 1.0);

      mPeriodicIO.diffWheelSpeeds = mKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed * scale, 0, rot));
    }
  }

  public void drive(ChassisSpeeds speeds) {
    mPeriodicIO.diffWheelSpeeds = mKinematics.toWheelSpeeds(speeds);
  }

  public void drive(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    // TODO: Implement feedforwards
    drive(speeds);
  }

  public void clearTurnPIDAccumulation() {
    mLeftPIDController.reset();
    mRightPIDController.reset();
  }

  public void setGyroAngleAdjustment(double angle) {
    mGyro.setAngleAdjustment(angle);
  }

  /** Update robot odometry. */
  public void updateOdometry() {
    mOdometry.update(mGyro.getRotation2d(), mLeftEncoder.getPosition(), mRightEncoder.getPosition());
  }

  /** Resets robot odometry. */
  public void resetOdometry(Pose2d pose) {
    // mPoseWasSet = true;
    mDrivetrainSimulator.setPose(pose);

    mOdometry.resetPosition(
        mGyro.getRotation2d(),
        mLeftEncoder.getPosition(),
        mRightEncoder.getPosition(),
        pose);
  }

  /** Check the current robot pose. */
  @AutoLogOutput
  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    mOdometry.resetPosition(
        mGyro.getRotation2d(),
        mLeftEncoder.getPosition(),
        mRightEncoder.getPosition(),
        pose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
        mLeftEncoder.getVelocity(),
        mRightEncoder.getVelocity());

    return mKinematics.toChassisSpeeds(wheelSpeeds);
  }

  public RobotConfig getRobotConfig() {
    return robotConfig;
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    mDrivetrainSimulator.setInputs(
        mLeftLeader.get() * RobotController.getInputVoltage(),
        mRightLeader.get() * RobotController.getInputVoltage());
    mDrivetrainSimulator.update(0.02);
  }

  @Override
  public void periodic() {
    var leftFeedforward = mLeftFeedforward.calculate(mPeriodicIO.diffWheelSpeeds.leftMetersPerSecond);
    var rightFeedforward = mRightFeedforward.calculate(mPeriodicIO.diffWheelSpeeds.rightMetersPerSecond);
    double leftOutput = mLeftPIDController.calculate(mLeftEncoder.getVelocity(),
        mPeriodicIO.diffWheelSpeeds.leftMetersPerSecond);
    double rightOutput = mRightPIDController.calculate(mRightEncoder.getVelocity(),
        mPeriodicIO.diffWheelSpeeds.rightMetersPerSecond);

    mPeriodicIO.leftVoltage = leftOutput + leftFeedforward;
    mPeriodicIO.rightVoltage = rightOutput + rightFeedforward;

    updateOdometry();

    m_field.setRobotPose(getPose());
  }

  @Override
  public void reset() {
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  @Override
  public void writePeriodicOutputs() {
    mLeftLeader.setVoltage(mPeriodicIO.leftVoltage);
    mRightLeader.setVoltage(mPeriodicIO.rightVoltage);
  }

  @Override
  public void stop() {
    mPeriodicIO.diffWheelSpeeds = new DifferentialDriveWheelSpeeds(0.0, 0.0);
  }

  @Override
  public void outputTelemetry() {
    putNumber("leftVelocitySetPoint", mPeriodicIO.diffWheelSpeeds.leftMetersPerSecond);
    putNumber("leftVelocityError", mPeriodicIO.diffWheelSpeeds.leftMetersPerSecond - mLeftEncoder.getVelocity());
    putNumber("rightVelocitySetPoint", mPeriodicIO.diffWheelSpeeds.rightMetersPerSecond);
    putNumber("rightVelocityError", mPeriodicIO.diffWheelSpeeds.rightMetersPerSecond - mRightEncoder.getVelocity());
    putNumber("leftVelocity", mLeftEncoder.getVelocity());
    putNumber("rightVelocity", mRightEncoder.getVelocity());
    putNumber("leftMeters", mLeftEncoder.getPosition());
    putNumber("rightMeters", mRightEncoder.getPosition());
    putNumber("Gyro", mGyro.getAngle());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return mSysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return mSysIdRoutine.dynamic(direction);
  }
}
