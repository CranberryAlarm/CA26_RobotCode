package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.BrakeTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;

public class PPTestMode extends AutoModeBase {
    public void queueTasks() {
        queueTask(new DriveTrajectoryTask("Note1"));
        queueTask(new DriveTrajectoryTask("Note1Back"));
        queueTask(new DriveTrajectoryTask("Note3"));
        queueTask(new DriveTrajectoryTask("Note3Back"));

        queueTask(new BrakeTask());
    }
}
