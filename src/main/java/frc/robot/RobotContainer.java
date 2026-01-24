
package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.units.measure.Distance;
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
  // Instatiate each subsystem
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem();
  private final HoodSubsystem m_hood = new HoodSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final HopperSubsystem m_hopper = new HopperSubsystem();
  private final KickerSubsystem m_kicker = new KickerSubsystem();

  // Create the Superstructure, passing in all relevant subsystem
  private final Superstructure m_superstructure = new Superstructure(m_shooter, m_turret, m_hood, m_intake, m_hopper, m_kicker);

  private final SendableChooser<Command> m_autoChooser;

  // Track current alliance for change detection
  private Alliance m_currentAlliance = Alliance.Red;

  /**
   * The container for the robot. Contains subsystems, I/O devices, and commands.
   */
  public RobotContainer() {
    // Configure bindings between controllers and robot commands
    configureBindings();

    // Create "Named" autonomous commands for PathPlanner/Choreo
    buildNamedAutoCommands();

    // Initialize alliance (default to red if not present)
    onAllianceChanged(getAlliance());

    // Set up trigger to detect alliance changes
    new Trigger(() -> Constants.getAlliance() != m_currentAlliance)
        .onTrue(Commands.runOnce(() -> onAllianceChanged(Constants.getAlliance())).ignoringDisable(true));

    // Triggers for auto aim/pass poses
    new Trigger(() -> isInAllianceZone())
        .onChange(Commands.runOnce(() -> onZoneChanged()).ignoringDisable(true));

    new Trigger(() -> isOnAllianceOutpostSide())
        .onChange(Commands.runOnce(() -> onZoneChanged()).ignoringDisable(true));

    if (!Robot.isReal() || true) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Have the autoChooser pull in all PathPlanner autos as options
    m_autoChooser = AutoBuilder.buildAutoChooser();

    // Set the default auto (do nothing)
    m_autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // Add a simple auto option to have the robot drive forward for 1 second then
    // stop
    m_autoChooser.addOption("Drive Forward", m_drivebase.driveForward().withTimeout(10));

    // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  /**
   * Configure bindings between Joysticks/Gamepads and commands
   */
  private void configureBindings() {
    // Set up controllers
    DriverControls.configure(ControllerConstants.kDriverControllerPort, m_drivebase, m_superstructure);
    OperatorControls.configure(ControllerConstants.kOperatorControllerPort, m_drivebase, m_superstructure);
    PoseControls.configure(ControllerConstants.kPoseControllerPort, m_drivebase);
  }

  /**
   * Create and register our "Named" autonomous Commands
   *
   * Named autonomous commands can be referenced from PathPlanner and Choreo
   * to trigger at specific points throughout paths:
   *
   * PathPlanner: https://pathplanner.dev/pplib-named-commands.html
   * Choreo: https://choreo.autos/usage/editing-paths/#pathplanner-interop
   */
  private void buildNamedAutoCommands() {
    // Add any auto commands to the NamedCommands here
    NamedCommands.registerCommand("ScoreCoral",
        Commands.runOnce(() -> System.out.println("Scoring Coral!"), m_drivebase)
            .andThen(Commands.waitSeconds(1))
            .withName("Auto.ScoreCoral"));

    NamedCommands.registerCommand("Dealgae",
        Commands.runOnce(() -> System.out.println("Dealgae!"), m_drivebase)
            .andThen(Commands.waitSeconds(1))
            .withName("Auto.Dealgae"));

    NamedCommands.registerCommand("driveBackwards",
        m_drivebase.driveBackwards().withTimeout(1)
            .withName("Auto.driveBackwards"));

    NamedCommands.registerCommand("driveForwards",
        m_drivebase.driveForward().withTimeout(2)
            .withName("Auto.driveForwards"));
  }

  /**
   * Get the active autonomous selection from Glass/SmartDashboard
   *
   * @return The currently selected autonomous command
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public SwerveDrive getSwerveDrive() {
    return m_drivebase.getSwerveDrive();
  }

  /**
   * Get the current robot pose (position and rotation), as reported by the drivebase
   *
   * @return Current robot pose
   */
  public Pose2d getRobotPose() {
    return m_drivebase.getPose();
  }

  /**
   * Get the position and aim direction of the shooter
   *
   * @return Current Pose3d (position and rotation) of the shooter
   */
  public Pose3d getAimDirection() {
    // Apply robot heading first, then turret/hood rotation on top
    Pose3d shooterPose = m_superstructure.getShooterPose();

    Pose3d pose = m_drivebase.getPose3d().plus(new Transform3d(
        shooterPose.getTranslation(), shooterPose.getRotation()));

    return pose;
  }

  /**
   * Get the field location that the shooter is aiming towards
   *
   * @return The field location the shooter is aiming towards
   */
  public Translation3d getAimPoint() {
    return m_superstructure.getAimPoint();
  }

  /**
   * Set the field location that the shooter should aim towards
   *
   * @param aimPoint Field location to aim towards
  */
  public void setAimPoint(Translation3d aimPoint) {
    m_superstructure.setAimPoint(aimPoint);
  }

  /**
   * Checks if robot is in it's own alliance zone
   *
   * @return true when the robot is within it's alliance zone, false otherwise
   */
  private boolean isInAllianceZone() {
    Alliance alliance = Constants.getAlliance();
    Distance blueZone = Inches.of(182);
    Distance redZone = Inches.of(469);

    if (alliance == Alliance.Blue && m_drivebase.getPose().getMeasureX().lt(blueZone)) {
      return true;
    } else if (alliance == Alliance.Red && m_drivebase.getPose().getMeasureX().gt(redZone)) {
      return true;
    }

    return false;
  }

  /**
   * Checks if robot is in the "Outpost" half of the field
   *
   * This is determining the Y location (left/right) of the robot
   *
   * @return true when the robot is on the Outpost side, false otherwise
   */
  private boolean isOnAllianceOutpostSide() {
    Alliance alliance = Constants.getAlliance();
    Distance midLine = Inches.of(158.84375);

    if (alliance == Alliance.Blue && m_drivebase.getPose().getMeasureY().lt(midLine)) {
      return true;
    } else if (alliance == Alliance.Red && m_drivebase.getPose().getMeasureY().gt(midLine)) {
      return true;
    }

    return false;
  }

  /**
   * Perform all necessary actions when the robot moves from one zone to another
   *
   * If the robot is within it's own alliance zone, aim at the hub to score.
   * Otherwise, aim at the appropriate corner of it's alliance zone for passing.
   */
  private void onZoneChanged() {
    if (isInAllianceZone()) {
      m_superstructure.setAimPoint(Constants.AimPoints.getAllianceHubPosition());
    } else {
      if (isOnAllianceOutpostSide()) {
        m_superstructure.setAimPoint(Constants.AimPoints.getAllianceOutpostPosition());
      } else {
        m_superstructure.setAimPoint(Constants.AimPoints.getAllianceFarSidePosition());
      }
    }
  }

  /**
   * Perform all necessary actions when the alliance changes.
   */
  private void onAllianceChanged(Alliance alliance) {
    m_currentAlliance = alliance;

    // Update aim point depending on which zone the robot is in
    onZoneChanged();

    System.out.println("Alliance changed to: " + alliance);
  }
}
