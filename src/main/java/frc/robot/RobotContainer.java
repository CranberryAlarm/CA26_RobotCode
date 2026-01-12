
package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.controls.DriverControls;
import frc.robot.controls.OperatorControls;
import frc.robot.controls.PoseControls;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final HopperSubsystem hopper = new HopperSubsystem();
  private final Superstructure superstructure = new Superstructure(null, null, null, intake, hopper);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, I/O devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    buildNamedAutoCommands();

    if (!Robot.isReal()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    // Set the default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // Add a simple auto option to have the robot drive forward for 1 second then
    // stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(10));

    // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Set up controllers
    DriverControls.configure(ControllerConstants.kDriverControllerPort, drivebase, superstructure);
    OperatorControls.configure(ControllerConstants.kOperatorControllerPort, drivebase, superstructure);
    PoseControls.configure(ControllerConstants.kPoseControllerPort, drivebase);
  }

  private void buildNamedAutoCommands() {
    // Add any auto commands to the NamedCommands here
    NamedCommands.registerCommand("ScoreCoral",
        Commands.runOnce(() -> System.out.println("Scoring Coral!"), drivebase)
            .andThen(Commands.waitSeconds(1))
            .withName("Auto.ScoreCoral"));

    NamedCommands.registerCommand("Dealgae",
        Commands.runOnce(() -> System.out.println("Dealgae!"), drivebase)
            .andThen(Commands.waitSeconds(1))
            .withName("Auto.Dealgae"));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public SwerveDrive getSwerveDrive() {
    return drivebase.getSwerveDrive();
  }

  public Pose2d getRobotPose() {
    return drivebase.getPose();
  }
}
