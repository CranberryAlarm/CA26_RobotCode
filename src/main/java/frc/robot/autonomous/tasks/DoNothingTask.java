package frc.robot.autonomous.tasks;

import frc.robot.RobotTelemetry;

public class DoNothingTask extends Task {
  @Override
  public void start() {
    RobotTelemetry.print("Starting do nothing auto...");
  }

  @Override
  public void update() {
    RobotTelemetry.print("Do nothing auto complete");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
