package frc.robot.autonomous.modes;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.Task;
import frc.robot.subsystems.Drivetrain;

public abstract class AutoModeBase {
  private ArrayList<Task> m_tasks;

  public AutoModeBase() {
    m_tasks = new ArrayList<>();
  }

  public Task getNextTask() {
    // Pop the first task off the list and return it
    try {
      return m_tasks.remove(0);
    } catch (IndexOutOfBoundsException ex) {
      return null;
    }
  }

  public void queueTask(Task task) {
    m_tasks.add(task);
  }

  public abstract void queueTasks();

  public void setStartingPose() {
    // Figure out the first PathPlanner path
    Pose2d startingPose = null;

    // Loop over the m_tasks and find the first DriveTrajectoryTask
    for (Task task : m_tasks) {
      if (task instanceof DriveTrajectoryTask) {
        // Set the starting pose to the starting pose of the first DriveTrajectoryTask
        startingPose = ((DriveTrajectoryTask) task).getStartingPose();
        break;
      }
    }

    // If there isn't one, default to something visible
    if (startingPose == null) {
      startingPose = Constants.Field.blueCenterPose2d;
    }

    Drivetrain m_drive = Drivetrain.getInstance();
    m_drive.resetOdometry(startingPose);
  };
}
