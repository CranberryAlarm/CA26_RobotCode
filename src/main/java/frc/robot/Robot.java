package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.CommandsLogging;
import frc.robot.util.maplesim.Arena2026Rebuilt;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private SimulatedArena m_arena;

  public Robot() {
    // Set the Project Name for Advantage Scope Logging
    Logger.recordMetadata("ProjectName", "CA_Ri3D_2026");

    // Setup Advantage Scope to publish to NetworkTables
    if (Constants.NT4_LOGGING) {
      Logger.addDataReceiver(new NT4Publisher());
    }

    // Setup Advantage Scope to publish to USB
    if (isReal() && Constants.USB_LOGGING) {
      // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new WPILOGWriter());
    }

    Logger.start();

    // Register CommandsLogging callbacks with CommandScheduler so Advantage Scope can log commands
    CommandScheduler.getInstance().onCommandInitialize(CommandsLogging::commandStarted);
    CommandScheduler.getInstance().onCommandFinish(CommandsLogging::commandEnded);
    CommandScheduler.getInstance().onCommandInterrupt(CommandsLogging::commandInterrupted);

    // Create the RobotContainer
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // Run the CommandScheduler logic (must be called in robotPeriodic())
    CommandScheduler.getInstance().run();

    // Log running commands and subsystem requirements
    CommandsLogging.logRunningCommands();
    CommandsLogging.logRequiredSubsystems();

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
    // Get the selected autonomous command from Glass/SmartDashboard
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Schedule the selected autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // Cancel any autonomous commands before starting teleop
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancel all commands before starting test periodic
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
    // Shut down the old arena instance first to release ownership of all bodies
    // (including the drivetrain) so they can be added to a new physics world
    SimulatedArena.getInstance().shutDown();

    // Create an assign a new arena instance for the 2026 field
    SimulatedArena.overrideInstance(new Arena2026Rebuilt());

    m_arena = SimulatedArena.getInstance();

    // Add the SwerveDrive simulation to the arena
    m_arena.addDriveTrainSimulation(m_robotContainer.getSwerveDrive().getMapleSimDrive().get());
  }

  @Override
  public void simulationPeriodic() {
    m_arena.simulationPeriodic();

    // Run the fuel game piece simulation
    Pose3d[] fuelPoses = m_arena.getGamePiecesArrayByType("Fuel");
    Logger.recordOutput("FieldSimulation/FuelPoses", fuelPoses);
  }
}
