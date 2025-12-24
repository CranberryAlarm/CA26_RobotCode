// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // Motor controllers
  private final SparkMax m_frontLeft = new SparkMax(DriveConstants.kFrontLeftMotorId, MotorType.kBrushless);
  private final SparkMax m_backLeft = new SparkMax(DriveConstants.kBackLeftMotorId, MotorType.kBrushless);
  private final SparkMax m_frontRight = new SparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
  private final SparkMax m_backRight = new SparkMax(DriveConstants.kBackRightMotorId, MotorType.kBrushless);

  // Motor controller groups for left and right sides
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_frontLeft, m_backLeft);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_frontRight, m_backRight);

  // Differential drive object
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Invert right side motors if needed
    // m_rightMotors.setInverted(true);
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
