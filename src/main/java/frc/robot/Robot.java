package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.CommandsLogging;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private SimulatedArena arena;

  public Robot() {
    Logger.recordMetadata("ProjectName", "CA_Ri3D_2026"); // Set a metadata value

    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

    if (isReal()) {
      // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    }

    Logger.start();

    // Register command logging callbacks with CommandScheduler
    CommandScheduler.getInstance().onCommandInitialize(CommandsLogging::commandStarted);
    CommandScheduler.getInstance().onCommandFinish(CommandsLogging::commandEnded);
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (interrupted, interrupting) -> {
              interrupting.ifPresent(
                  interrupter -> CommandsLogging.runningInterrupters.put(interrupter, interrupted));
              CommandsLogging.commandEnded(interrupted);
            });

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Log running commands and subsystem requirements
    CommandsLogging.logRunningCommands();
    CommandsLogging.logRequiredSubsystems();

    if (Robot.isSimulation()) {
      Pose3d[] algaePoses = arena.getGamePiecesArrayByType("Algae");
      Logger.recordOutput("FieldSimulation/AlgaePoses", algaePoses);

      Pose3d[] coralPoses = arena.getGamePiecesArrayByType("Coral");
      Logger.recordOutput("FieldSimulation/CoralPoses", coralPoses);
    }

    Logger.recordOutput("FieldSimulation/RobotPose", m_robotContainer.getRobotPose());
    Logger.recordOutput("FieldSimulation/TargetPose",
        m_robotContainer.getSwerveDrive().field.getObject("targetPose").getPose());
    Logger.recordOutput("FieldSimulation/AimDirection", m_robotContainer.getAimDirection());
    Logger.recordOutput("FieldSimulation/AimTarget", new Pose3d(m_robotContainer.getAimPoint(), Rotation3d.kZero));
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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

    // arena.addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(14, 1)));
  }

  @Override
  public void simulationPeriodic() {
    arena.simulationPeriodic();
  }
}
