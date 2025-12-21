package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.BrakeTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;

public class LongCook extends AutoModeBase {
    public void queueTasks() {
        queueTask(new DriveTrajectoryTask("Long1"));

        queueTask(new BrakeTask());
    }
}
