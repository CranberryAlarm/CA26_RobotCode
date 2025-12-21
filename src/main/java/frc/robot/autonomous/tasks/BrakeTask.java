package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Drivetrain;

public class BrakeTask extends Task {
  private Drivetrain m_drive;

  public BrakeTask() {
    m_drive = Drivetrain.getInstance();
  }

  @Override
  public void start() {
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void done() {
    m_drive.drive(0, 0);
  }
}
