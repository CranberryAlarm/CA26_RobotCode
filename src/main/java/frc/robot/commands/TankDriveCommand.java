// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TankDriveCommand extends Command {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_leftSpeed;
  private final DoubleSupplier m_rightSpeed;

  /**
   * Creates a new TankDriveCommand.
   *
   * @param drive      The drive subsystem to use
   * @param leftSpeed  A supplier for the left side speed
   * @param rightSpeed A supplier for the right side speed
   */
  public TankDriveCommand(DriveSubsystem drive, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    m_drive = drive;
    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    m_drive.tankDrive(m_leftSpeed.getAsDouble(), m_rightSpeed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }
}
