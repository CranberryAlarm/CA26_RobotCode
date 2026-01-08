
package frc.robot;

import java.io.File;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.controls.DriverControls;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, I/O devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    buildNamedAutoCommands();
    DriverStation.silenceJoystickConnectionWarning(true);

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
    DriverControls.configure(ControllerConstants.kDriverControllerPort, drivebase, null);
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

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public SwerveDrive getSwerveDrive() {
    return drivebase.getSwerveDrive();
  }

  public Pose2d getRobotPose() {
    return drivebase.getPose();
  }

  public SwerveDriveSimulation getSwerveDriveSimulation() {
    return drivebase.getSwerveDrive().getMapleSimDrive().orElseThrow();
  }

  public Command fireAlgae() {
    return Commands.runOnce(() -> {
      System.err.println("FIRE!");

      SimulatedArena arena = SimulatedArena.getInstance();

      // Translation2d robotPosition,
      // Translation2d shooterPositionOnRobot,
      // ChassisSpeeds chassisSpeeds,
      // Rotation2d shooterFacing,
      // Distance initialHeight,
      // LinearVelocity launchingSpeed,
      // Angle shooterAngle

      ReefscapeAlgaeOnFly algae = new ReefscapeAlgaeOnFly(
          drivebase.getPose().getTranslation(),
          new Translation2d(),
          drivebase.getSwerveDrive().getRobotVelocity().times(-1),
          drivebase.getSwerveDrive().getOdometryHeading(),
          Distance.ofBaseUnits(1, Feet),
          LinearVelocity.ofBaseUnits(5, FeetPerSecond),
          Angle.ofBaseUnits(45, Degrees));

      // Configure callbacks to visualize the flight trajectory of the projectile
      algae.withProjectileTrajectoryDisplayCallBack(
          // Callback for when the note will eventually hit the target (if configured)
          (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileSuccessfulShot",
              pose3ds.toArray(Pose3d[]::new)),
          // Callback for when the note will eventually miss the target, or if no target
          // is configured
          (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileUnsuccessfulShot",
              pose3ds.toArray(Pose3d[]::new)));

      arena.addGamePieceProjectile(algae);
    }).withName("Fire.Algae");
  }
}
