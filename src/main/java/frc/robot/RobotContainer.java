
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
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.feeder.Hopper;
import frc.robot.subsystems.feeder.Kicker;
import frc.robot.subsystems.feeder.RollerIONova;
import frc.robot.subsystems.feeder.RollerIOSim;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivotIOSim;
import frc.robot.subsystems.intake.IntakePivotIOSparkMax;
import frc.robot.subsystems.intake.IntakeRollerIONova;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOSparkMax;
import swervelib.SwerveDrive;

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  // IO-based subsystems with hardware/sim implementations
  private final Shooter shooter;
  private final Turret turret;
  private final Hood hood;
  private final Intake intake;
  private final Hopper hopper;
  private final Kicker kicker;

  private final Superstructure superstructure;

  private final SendableChooser<Command> autoChooser;

  // Track current alliance for change detection
  private Alliance currentAlliance = Alliance.Red;

  /**
   * The container for the robot. Contains subsystems, I/O devices, and commands.
   */
  public RobotContainer() {
    // Create subsystems with appropriate IO implementations based on real/sim
    if (Robot.isReal()) {
      // Real hardware implementations
      shooter = new Shooter(new ShooterIOSparkMax());
      turret = new Turret(new TurretIOSparkMax());
      hood = new Hood(new HoodIOSim()); // Hood hardware not implemented yet
      intake = new Intake(
          new IntakeRollerIONova(),
          new IntakePivotIOSparkMax());
      hopper = new Hopper(
          new RollerIONova(Constants.HopperConstants.kHopperMotorId, true, true, 40));
      kicker = new Kicker(
          new RollerIONova(Constants.KickerConstants.kKickerMotorId, true, true, 20));
    } else {
      // Simulation implementations
      shooter = new Shooter(new ShooterIOSim());
      turret = new Turret(new TurretIOSim());
      hood = new Hood(new HoodIOSim());
      intake = new Intake(new IntakeRollerIOSim(), new IntakePivotIOSim());
      hopper = new Hopper(new RollerIOSim());
      kicker = new Kicker(new RollerIOSim());
    }

    superstructure = new Superstructure(shooter, turret, hood, intake, hopper, kicker);

    // Configure the trigger bindings
    configureBindings();
    buildNamedAutoCommands();

    // Initialize alliance (default to red if not present)
    onAllianceChanged(getAlliance());

    // Set up trigger to detect alliance changes
    new Trigger(() -> getAlliance() != currentAlliance)
        .onTrue(Commands.runOnce(() -> onAllianceChanged(getAlliance())).ignoringDisable(true));

    // Triggers for auto aim/pass poses
    new Trigger(() -> isInAllianceZone())
        .onChange(Commands.runOnce(() -> onZoneChanged()).ignoringDisable(true));

    new Trigger(() -> isOnAllianceOutpostSide())
        .onChange(Commands.runOnce(() -> onZoneChanged()).ignoringDisable(true));

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

  private boolean isInAllianceZone() {
    Alliance alliance = getAlliance();
    Distance blueZone = Inches.of(182);
    Distance redZone = Inches.of(469);

    if (alliance == Alliance.Blue && drivebase.getPose().getMeasureX().lt(blueZone)) {
      return true;
    } else if (alliance == Alliance.Red && drivebase.getPose().getMeasureX().gt(redZone)) {
      return true;
    }

    return false;
  }

  private boolean isOnAllianceOutpostSide() {
    Alliance alliance = getAlliance();
    Distance midLine = Inches.of(158.84375);

    if (alliance == Alliance.Blue && drivebase.getPose().getMeasureY().lt(midLine)) {
      return true;
    } else if (alliance == Alliance.Red && drivebase.getPose().getMeasureY().gt(midLine)) {
      return true;
    }

    return false;
  }

  private void onZoneChanged() {
    if (isInAllianceZone()) {
      superstructure.setAimPoint(Constants.AimPoints.getAllianceHubPosition());
    } else {
      if (isOnAllianceOutpostSide()) {
        superstructure.setAimPoint(Constants.AimPoints.getAllianceOutpostPosition());
      } else {
        superstructure.setAimPoint(Constants.AimPoints.getAllianceFarSidePosition());
      }
    }
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
