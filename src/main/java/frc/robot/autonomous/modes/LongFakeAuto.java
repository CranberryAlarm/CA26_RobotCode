package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.BrakeTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;

public class LongFakeAuto extends AutoModeBase {
    public void queueTasks() {
        queueTask(new DriveTrajectoryTask("LongFake1"));
        queueTask(new DriveTrajectoryTask("LongFake1Back"));
        queueTask(new DriveTrajectoryTask("LongFake2"));
        queueTask(new DriveTrajectoryTask("LongFake2Back"));

        queueTask(new BrakeTask());
    }
}
