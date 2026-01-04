package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private SimulatedArena arena;

  public Robot() {
    Logger.recordMetadata("ProjectName", "CA_Ri3D_2026"); // Set a metadata value

    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    } else {
      // setUseTiming(false); // Run as fast as possible
      // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from
      // // AdvantageScope (or prompt the user)
      // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
      // "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or
    // metadata values may
    // be added.

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (Robot.isSimulation()) {
      // Get the positions of the notes (both on the field and in the air)
      Pose3d[] notesPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Note");
      // Publish to telemetry using AdvantageKit
      Logger.recordOutput("FieldSimulation/NotesPositions", notesPoses);

      Logger.recordOutput("FieldSimulation/RobotPose", m_robotContainer.getRobotPose());
      Logger.recordOutput("FieldSimulation/TargetPose",
          m_robotContainer.getSwerveDrive().field.getObject("targetPose").getPose());
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    // TODO: Add this back when we need real autos
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
    arena = SimulatedArena.getInstance();

    arena.addGamePiece(new CrescendoNoteOnField(new Translation2d(3, 3)));
  }

  @Override
  public void simulationPeriodic() {
    arena.simulationPeriodic();
  }
}
