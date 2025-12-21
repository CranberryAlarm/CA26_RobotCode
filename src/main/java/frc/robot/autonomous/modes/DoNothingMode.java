package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.DoNothingTask;

public class DoNothingMode extends AutoModeBase {
  public void queueTasks() {
    queueTask(new DoNothingTask());
  }
}
