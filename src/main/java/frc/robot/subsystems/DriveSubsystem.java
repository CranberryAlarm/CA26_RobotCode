// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private static final double kTrackWidth = Units.inchesToMeters(20.75);
  private static final double kWheelRadius = Units.inchesToMeters(3.0);
  private static final double kGearRatio = 10.71;
  private static final double kMetersPerRev = (2.0 * Math.PI * kWheelRadius) / kGearRatio;

  // Motor controllers
  private final SparkMax m_frontLeft = new SparkMax(DriveConstants.kFrontLeftMotorId, MotorType.kBrushless);
  private final SparkMax m_backLeft = new SparkMax(DriveConstants.kBackLeftMotorId, MotorType.kBrushless);
  private final SparkMax m_frontRight = new SparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
  private final SparkMax m_backRight = new SparkMax(DriveConstants.kBackRightMotorId, MotorType.kBrushless);

  // Differential drive object
  private final DifferentialDrive m_drive = new DifferentialDrive(m_frontLeft, m_frontRight);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    var globalConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kCoast);

    var encoderConfig = new SparkMaxConfig().encoder
        // The "native units" for the SparkMax is motor rotations:
        // Conversion factor = (distance traveled per motor shaft rotation)
        .positionConversionFactor(kMetersPerRev)

        // The "native units" for the SparkMax is RPM:
        // Conversion factor = (distance traveled per motor shaft rotation) / (60
        // seconds)
        .velocityConversionFactor(kMetersPerRev / 60);

    var leftLeaderConfig = new SparkMaxConfig()
        .apply(globalConfig)
        .apply(encoderConfig);

    var leftFollowerConfig = new SparkMaxConfig()
        .apply(globalConfig)
        .follow(m_frontLeft);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    var rightLeaderConfig = new SparkMaxConfig()
        .apply(globalConfig)
        .apply(encoderConfig)
        .inverted(true);

    var rightFollowerConfig = new SparkMaxConfig()
        .apply(globalConfig)
        .apply(rightLeaderConfig)
        .follow(m_frontRight);

    m_frontLeft.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_backLeft.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_frontRight.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_backRight.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Drive the robot using tank drive controls.
   *
   * @param leftSpeed  The speed for the left side of the robot (-1.0 to 1.0)
   * @param rightSpeed The speed for the right side of the robot (-1.0 to 1.0)
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Drive the robot using arcade drive controls.
   *
   * @param speed    The forward/backward speed (-1.0 to 1.0)
   * @param rotation The rotation speed (-1.0 to 1.0)
   */
  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  /** Stops the drive motors. */
  public void stop() {
    m_drive.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
