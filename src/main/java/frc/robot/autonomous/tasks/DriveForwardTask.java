package frc.robot.autonomous.tasks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotTelemetry;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardTask extends Task {
  private Drivetrain m_drive;
  private double m_targetDistance;
  private double m_speed;
  private Pose2d m_startPose;

  private Timer m_runningTimer = new Timer();
  private double m_lastTime = 0;

  public DriveForwardTask(double distance, double speed) {
    m_drive = Drivetrain.getInstance();
    m_targetDistance = distance;
    m_speed = speed;
  }

  @Override
  public void start() {
    m_runningTimer.reset();
    m_runningTimer.start();

    m_startPose = m_drive.getPose();
  }

  @Override
  public void update() {
    m_drive.drive(m_speed, 0);
  }

  @Override
  public void updateSim() {
    // This simulates the robot driving in the positive x direction
    if (!RobotBase.isReal()) {
      Pose2d currentPose = m_drive.getPose();

      // Move "forward", based on the robot's current rotation
      double newX = currentPose.getX()
          - m_speed * (m_runningTimer.get() - m_lastTime) * Math.cos(currentPose.getRotation().getRadians());
      double newY = currentPose.getY()
          - m_speed * (m_runningTimer.get() - m_lastTime) * Math.sin(currentPose.getRotation().getRadians());

      Pose2d newPose = new Pose2d(
          newX,
          newY,
          currentPose.getRotation());

      m_drive.setPose(newPose);
      m_lastTime = m_runningTimer.get();
    }
  }

  @Override
  public boolean isFinished() {
    Pose2d relativePose = m_startPose.relativeTo(m_drive.getPose());
    return Math.hypot(relativePose.getX(), relativePose.getY()) >= m_targetDistance;
  }

  @Override
  public void done() {
    RobotTelemetry.print("Auto driving done");
    m_drive.drive(0, 0);
  }
}
