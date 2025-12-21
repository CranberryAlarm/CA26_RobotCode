package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.BrakeTask;
import frc.robot.autonomous.tasks.DriveForwardTask;

public class DefaultMode extends AutoModeBase {
  public void queueTasks() {
    // queueTask(new ParallelTask(
    // new IntakeTask(IntakeState.NONE, PivotTarget.STOW),
    // new ShooterTask(3000),
    // new WaitTask(2.0)));

    // queueTask(new ParallelTask(
    // new IntakeTask(IntakeState.EJECT, PivotTarget.STOW),
    // new WaitTask(1.0)));

    // queueTask(new ParallelTask(
    // new IntakeTask(IntakeState.INTAKE, PivotTarget.GROUND),
    // new WaitTask(0.1)));

    queueTask(new DriveForwardTask(1.3, 0.4));

    // queueTask(new ParallelTask(
    // new IntakeTask(IntakeState.PULSE, PivotTarget.STOW),
    // new WaitTask(0.1)));

    queueTask(new DriveForwardTask(1.2, -0.4));

    // queueTask(new ParallelTask(
    // new IntakeTask(IntakeState.EJECT, PivotTarget.STOW),
    // new WaitTask(2.0)));

    // queueTask(new ParallelTask(
    // new IntakeTask(IntakeState.NONE, PivotTarget.STOW),
    // new WaitTask(2.0),
    // new ShooterTask(0)));

    queueTask(new BrakeTask());
  }
}
