
package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.controls.DriverControls;
import frc.robot.controls.OperatorControls;
import frc.robot.controls.PoseControls;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import swervelib.SwerveDrive;

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final TurretSubsystem turret = new TurretSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final HopperSubsystem hopper = new HopperSubsystem();
  private final KickerSubsystem kicker = new KickerSubsystem();
  private final HoodSubsystem hood = new HoodSubsystem();

  private final Superstructure superstructure = new Superstructure(shooter, turret, hood, intake, hopper, kicker);

  private final SendableChooser<Command> autoChooser;

  // Track current alliance for change detection
  private Alliance currentAlliance = Alliance.Red;

  /**
   * The container for the robot. Contains subsystems, I/O devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    buildNamedAutoCommands();

    // Initialize alliance (default to red if not present)
    onAllianceChanged(getAlliance());

    // Set up trigger to detect alliance changes
    new Trigger(() -> getAlliance() != currentAlliance)
        .onTrue(Commands.runOnce(() -> onAllianceChanged(getAlliance())).ignoringDisable(true));

    if (!Robot.isReal() || true) {
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

    NamedCommands.registerCommand("driveBackwards",
        drivebase.driveBackwards().withTimeout(1)
            .withName("Auto.driveBackwards"));

    NamedCommands.registerCommand("driveForwards",
        drivebase.driveForward().withTimeout(2)
            .withName("Auto.driveForwards"));
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

  public Pose3d getAimDirection() {
    // Apply robot heading first, then turret/hood rotation on top
    Pose3d shooterPose = superstructure.getShooterPose();

    var pose = drivebase.getPose3d().plus(new Transform3d(
        shooterPose.getTranslation(), shooterPose.getRotation()));

    return pose;
  }

  public Translation3d getAimPoint() {
    return superstructure.getAimPoint();
  }

  public void setAimPoint(Translation3d aimPoint) {
    superstructure.setAimPoint(aimPoint);
  }

  private Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red);
  }

  private void onAllianceChanged(Alliance alliance) {
    currentAlliance = alliance;

    // Update aim point based on alliance
    if (alliance == Alliance.Blue) {
      superstructure.setAimPoint(Constants.AimPoints.BLUE_HUB.value);
    } else {
      superstructure.setAimPoint(Constants.AimPoints.RED_HUB.value);
    }

    System.out.println("Alliance changed to: " + alliance);
  }
}
