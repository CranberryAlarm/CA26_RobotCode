package frc.robot.autonomous.tasks;

public abstract class Task {
  public abstract void start();

  public abstract void update();

  public void updateSim() {
  }

  public abstract boolean isFinished();

  public void done() {
  };
}
