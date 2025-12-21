package frc.robot.autonomous.tasks;

public class ParallelTask extends Task {
  private Task[] m_tasks;
  private boolean[] m_finished;
  private boolean m_allFinished = false;

  public ParallelTask(Task... tasks) {
    this.m_tasks = tasks;
    m_finished = new boolean[tasks.length];
  }

  @Override
  public void start() {
    for (Task task : m_tasks) {
      task.start();
    }
  }

  @Override
  public void update() {
    for (int i = 0; i < m_tasks.length; i++) {
      if (!m_finished[i]) {
        m_tasks[i].update();
        if (m_tasks[i].isFinished()) {
          m_tasks[i].done();
          m_finished[i] = true;
        }
      }
    }
    m_allFinished = true;
    for (boolean b : m_finished) {
      if (!b) {
        m_allFinished = false;
      }
    }
  }

  @Override
  public void updateSim() {
    for (Task task : m_tasks) {
      if (!task.isFinished()) {
        task.updateSim();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return m_allFinished;
  }

  @Override
  public void done() {
    // for (Task task : m_tasks) {
    // task.done();
    // }
  }
}
