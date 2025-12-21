// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autonomous.AutoRunner;
import frc.robot.autonomous.tasks.Task;
import frc.robot.controls.controllers.DriverController;
import frc.robot.controls.controllers.OperatorController;
import frc.robot.simulation.Field;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.leds.LEDs;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  // Controller
  private final DriverController m_driverController = new DriverController(0, true, true);
  private final OperatorController m_operatorController = new OperatorController(1, true, true);
  private final GenericHID sysIdController = new GenericHID(2);

  // private final SlewRateLimiter m_speedLimiter = new
  // SlewRateLimiter(Drivetrain.kMaxAcceleration);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Math.PI * 8);

  // Robot subsystems
  private List<Subsystem> m_allSubsystems = new ArrayList<>();
  private final Drivetrain m_drive = Drivetrain.getInstance();
  private final Coral m_coral = Coral.getInstance();
  private final Algae m_algae = Algae.getInstance();
  private final Elevator m_elevator = Elevator.getInstance();

  public final LEDs m_leds = LEDs.getInstance();

  // Auto stuff
  private Task m_currentTask;
  private AutoRunner m_autoRunner = AutoRunner.getInstance();

  // Simulation stuff
  private final Field m_field = Field.getInstance();

  /**
   * This function is run when the robot is first started up.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();

    setupLogging();

    // Add all subsystems to the list
    // m_allSubsystems.add(m_compressor);
    m_allSubsystems.add(m_drive);
    m_allSubsystems.add(m_coral);
    m_allSubsystems.add(m_algae);
    m_allSubsystems.add(m_elevator);

    m_allSubsystems.add(m_leds);

    // Set up the Field2d object for simulation
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void robotPeriodic() {
    m_allSubsystems.forEach(subsystem -> subsystem.periodic());
    m_allSubsystems.forEach(subsystem -> subsystem.writePeriodicOutputs());
    m_allSubsystems.forEach(subsystem -> subsystem.outputTelemetry());
    m_allSubsystems.forEach(subsystem -> subsystem.writeToLog());

    updateSim();

    // Used by sysid
    if (this.isTestEnabled()) {
      CommandScheduler.getInstance().run();
    }
  }

  @Override
  public void autonomousInit() {
    m_currentTask = m_autoRunner.getNextTask();

    // Start the first task
    if (m_currentTask != null) {
      m_currentTask.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // If there is a current task, run it
    if (m_currentTask != null) {
      // Run the current task
      m_currentTask.update();
      m_currentTask.updateSim();

      // If the current task is finished, get the next task
      if (m_currentTask.isFinished()) {
        m_currentTask.done();
        m_currentTask = m_autoRunner.getNextTask();

        // Start the next task
        if (m_currentTask != null) {
          m_currentTask.start();
        }
      }
    }
  }

  @Override
  public void teleopInit() {
    m_leds.breathe();
  }

  double speed = 0;
  boolean scorePressed = false;

  @Override
  public void teleopPeriodic() {
    double maxSpeed = m_driverController.getWantsSpeedMode() ? Drivetrain.kMaxBoostSpeed : Drivetrain.kMaxSpeed;

    // double xSpeed = m_speedLimiter.calculate(m_driverController.getForwardAxis()
    // * maxSpeed);
    double xSpeed = m_driverController.getForwardAxis() * maxSpeed;

    // m_drive.slowMode(m_driverController.getWantsSlowMode());
    // m_drive.speedMode(m_driverController.getWantsSpeedMode());
    double rot = m_rotLimiter.calculate(m_driverController.getTurnAxis() * Drivetrain.kMaxAngularSpeed);

    m_drive.drive(xSpeed, rot);

    // FINAL DRIVER CONTROLS
    if (m_driverController.getWantsScoreCoral()) {
      scorePressed = true;

      if (m_elevator.getState() == Elevator.ElevatorState.STOW) {
        m_coral.scoreL1();
      } else {
        m_coral.scoreL24();
      }
    } else if (scorePressed) {
      scorePressed = false;

      m_elevator.goToElevatorStow();
      m_coral.intake();
    } else if (m_driverController.getWantsScoreAlgae()) {
      m_algae.score();
    } else if (m_driverController.getWantsGroundAlgae()) {
      m_algae.groundIntake();
    }

    // FINAL OPERATOR CONTROLS
    if (m_operatorController.getWantsElevatorStow()) {
      m_elevator.goToElevatorStow();
      m_algae.stow();
    } else if (m_operatorController.getWantsElevatorL2()) {
      m_elevator.goToElevatorL2();
      m_algae.stow();
    } else if (m_operatorController.getWantsElevatorL3()) {
      m_elevator.goToElevatorL3();
      m_algae.stow();
    } else if (m_operatorController.getWantsElevatorL4()) {
      m_elevator.goToElevatorL4();
      m_algae.stow();
    } else if (m_operatorController.getWantsA1()) {
      m_elevator.goToAlgaeLow();
      m_algae.grabAlgae();
    } else if (m_operatorController.getWantsA2()) {
      m_elevator.goToAlgaeHigh();
      m_algae.grabAlgae();
    } else if (m_operatorController.getWantsStopAlgae()) {
      m_algae.stopAlgae();
      m_algae.stow();
    } else if (m_operatorController.getWantsGroundAlgae()) {
      m_algae.groundIntake();
    } else if (m_operatorController.getWantsCoralIntake()) {
      m_coral.intake();
    }

    // if (m_driverController.getWantsScoreCoral()) {
    // if (m_elevator.getState() == Elevator.ElevatorState.STOW) {
    // m_coral.scoreL1();
    // } else {
    // m_coral.scoreL24();
    // }
    // } else if (m_driverController.getWantsIntakeCoral()) {
    // m_coral.intake();
    // m_elevator.goToElevatorStow();
    // }

    // if (m_operatorController.getWantsElevatorReset() ||
    // m_driverController.getWantsElevatorReset()) {
    // RobotTelemetry.print("Resetting elevator");
    // m_elevator.reset();
    // }
  }

  @Override
  public void disabledInit() {
    m_leds.rainbow();
    // m_leds.setColor(Color.kRed);

    speed = 0;
    m_allSubsystems.forEach(subsystem -> subsystem.stop());

    // TODO: reset the auto state stuff if we're in dev mode
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    if (sysIdController.getRawButtonPressed(1)) {
      // A
      m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).schedule();
    } else if (sysIdController.getRawButtonPressed(2)) {
      // B
      m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).schedule();
    } else if (sysIdController.getRawButtonPressed(3)) {
      // X
      m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward).schedule();
    } else if (sysIdController.getRawButtonPressed(4)) {
      // Y
      m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse).schedule();
    } else if (sysIdController.getRawButtonPressed(8)) {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  private void updateSim() {
    // Update the odometry in the sim.
    m_field.setRobotPose(m_drive.getPose());
  }

  @SuppressWarnings("resource")
  private void setupLogging() {
    Logger.recordMetadata("ProjectName", "Flipside"); // Set a metadata value

    if (isReal()) {
      new WPILOGWriter(); // Log to the RoboRIO

      // TODO: Add the next line back with a USB stick
      // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kCTRE); // Enables power distribution logging
    }

    Logger.start();
  }
}
