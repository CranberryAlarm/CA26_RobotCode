
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();

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
        m_driveSubsystem.arcadeDrive(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getRightX()));

    m_coralSubsystem.setDefaultCommand(
        m_coralSubsystem.set(0));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.a().onTrue(m_algaeSubsystem.setAngle(Degrees.of(3)));
    m_driverController.b().onTrue(m_algaeSubsystem.setAngle(Degrees.of(-90)));

    m_driverController.y().onTrue(m_coralSubsystem.scoreCoral());
    m_driverController.x().onTrue(m_coralSubsystem.intakeCoral());

    m_driverController.start().onTrue(m_coralSubsystem.sysId());
  }

  // public Command getAutonomousCommand() {
  // // An example command will be run in autonomous
  // // return Autos.exampleAuto(m_algaeSubsystem);
  // return new Command() {}; //Autos.exampleAuto(m_algaeSubsystem);
  // }
}
