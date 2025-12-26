
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, I/O devices, and commands.
   */
  public RobotContainer() {
    // Configure the default command for the drive subsystem
    // Tank drive using left Y-axis and right Y-axis
    m_driveSubsystem.setDefaultCommand(
        new TankDriveCommand(
            m_driveSubsystem,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getRightY()));

    m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.setAngle(Degrees.of(0)));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Schedule `setAngle` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.a().whileTrue(m_algaeSubsystem.setAngle(Degrees.of(-5)));
    m_driverController.b().whileTrue(m_algaeSubsystem.setAngle(Degrees.of(15)));

    // Schedule `set` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.x().whileTrue(m_algaeSubsystem.set(0.3));
    m_driverController.y().whileTrue(m_algaeSubsystem.set(-0.3));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_algaeSubsystem);
  }
}
