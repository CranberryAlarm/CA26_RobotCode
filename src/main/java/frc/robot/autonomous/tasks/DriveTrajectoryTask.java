package frc.robot.autonomous.tasks;

import java.nio.file.Path;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotTelemetry;
import frc.robot.subsystems.Drivetrain;

public class DriveTrajectoryTask extends Task {
  private Drivetrain m_drive;
  private PathPlannerTrajectory m_autoTrajectory;
  private boolean m_isFinished = false;
  private String m_smartDashboardKey = "DriveTrajectoryTask/";
  private PathPlannerPath m_autoPath = null;

  private final Timer m_runningTimer = new Timer();
  // private PPRamseteController m_driveController;
  private PPLTVController m_driveController;

  public DriveTrajectoryTask(String pathName) {
    m_drive = Drivetrain.getInstance();
    Path filePath = null;

    try {
      if (RobotBase.isReal()) {
        RobotTelemetry.print("Running on the robot!");
        filePath = Filesystem.getDeployDirectory().toPath().resolve("paths/" + pathName);
      } else {
        RobotTelemetry.print("Running in simulation!");
        filePath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/paths/" + pathName);
      }

      RobotTelemetry.print("Loading path from:\n" + filePath.toString());
      m_autoPath = PathPlannerPath.fromPathFile(pathName);
      // RobotTelemetry.print(m_autoPath.numPoints());

      if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
        RobotTelemetry.print("Translating path for Red Alliance!");
        m_autoPath = m_autoPath.flipPath();
      }
    } catch (Exception ex) {
      DriverStation.reportError("Unable to load PathPlanner trajectory: " + pathName, ex.getStackTrace());
      m_isFinished = true;
    }

    m_autoTrajectory = new PathPlannerTrajectory(
        m_autoPath,
        new ChassisSpeeds(),
        m_drive.getPose().getRotation(),
        m_drive.getRobotConfig());

    if (m_autoPath.isReversed()) {
      RobotTelemetry.print("===== PATH IS REVERSED =====");
    }

    double rotationErrorTolerance = Math.pow(m_autoPath.getGlobalConstraints().maxVelocityMPS(), 1.25) * 0.25;
    // double translationErrorTolerance =
    // m_autoPath.getGlobalConstraints().getMaxVelocityMps() * 0.0625;

    m_driveController = new PPLTVController(
        VecBuilder.fill(0.25, 0.25, 1.0),
        VecBuilder.fill(1.0, 2.0),
        0.02,
        m_autoPath.getGlobalConstraints().maxVelocityMPS());

    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
    // m_driveController = new PPRamseteController(2, 0.7);
  }

  @Override
  public void start() {
    m_isFinished = false;

    Pose2d currentPose = m_drive.getPose();
    ChassisSpeeds currentSpeeds = m_drive.getCurrentSpeeds();

    m_driveController.reset(currentPose, currentSpeeds);

    Rotation2d currentHeading = currentPose.getRotation();
    Rotation2d targetHeading;
    if (m_autoPath.isReversed()) {
      targetHeading = m_autoPath.getPoint(0).position.minus(m_autoPath.getPoint(1).position).getAngle();
    } else {
      targetHeading = m_autoPath.getPoint(1).position.minus(m_autoPath.getPoint(0).position).getAngle();
    }
    Rotation2d headingError = currentHeading.minus(targetHeading);

    boolean onHeading = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond) < 0.25
        && Math.abs(headingError.getDegrees()) < 1;

    boolean onStartPose = currentPose.getTranslation().getDistance(m_autoPath.getPoint(0).position) < 0.25;

    boolean shouldReplan = !onStartPose || !onHeading;

    Logger.recordOutput("Auto/DriveTrajectory/shouldReplan", shouldReplan);

    if (shouldReplan) {
      // TODO: maybe do this later?
      // ...
      // Or maybe not...
      // replanPath(currentPose, currentSpeeds);
    }

    // DEBUG Trajectory //////////////////////////////////////////////
    // Trajectory adjustedTrajectory = TrajectoryGenerator.generateTrajectory(
    // m_autoPath.getPathPoses(),
    // new TrajectoryConfig(
    // m_autoPath.getGlobalConstraints().getMaxVelocityMps(),
    // m_autoPath.getGlobalConstraints().getMaxAccelerationMpsSq()));
    // Logger.recordOutput("Auto/DriveTrajectory/TargetTrajectory",
    // adjustedTrajectory);
    // if (shouldReplan) {
    // Logger.recordOutput("Auto/DriveTrajectory/ReplannedTrajectory",
    // adjustedTrajectory);
    // }
    /////////////////////////////////////////////////////////////////

    m_drive.clearTurnPIDAccumulation();
    RobotTelemetry.print("Running path for " + DriverStation.getAlliance().toString());

    m_runningTimer.reset();
    m_runningTimer.start();
  }

  @Override
  public void update() {
    PathPlannerTrajectoryState goal = m_autoTrajectory.sample(m_runningTimer.get());
    if (m_autoPath.isReversed()) {
      goal = goal.reverse();
    }
    ChassisSpeeds chassisSpeeds = m_driveController.calculateRobotRelativeSpeeds(m_drive.getPose(), goal);

    m_drive.drive(chassisSpeeds);

    m_isFinished |= m_runningTimer.hasElapsed(m_autoTrajectory.getTotalTimeSeconds());

    Logger.recordOutput("Auto/DriveTrajectory/TargetPose", goal.pose);
    Logger.recordOutput("Auto/DriveTrajectory/CurrentPose", m_drive.getPose());

    SmartDashboard.putNumber(m_smartDashboardKey + "vx", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber(m_smartDashboardKey + "vy", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber(m_smartDashboardKey + "vr", chassisSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void updateSim() {
    if (!RobotBase.isReal()) {
      PathPlannerTrajectoryState autoState = m_autoTrajectory
          .sample(m_runningTimer.get());

      m_drive.setPose(autoState.pose);
    }
  }

  public Pose2d getStartingPose() {
    return m_autoPath.getStartingDifferentialPose();
  }

  @Override
  public boolean isFinished() {
    return m_isFinished;
  }

  @Override
  public void done() {
    RobotTelemetry.print("Auto trajectory done");
    m_drive.drive(0, 0);
  }
}
