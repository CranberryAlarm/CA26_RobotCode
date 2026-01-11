package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex flywheelMotor;

  private final double INTAKE_SPEED = 0.5;

  public IntakeSubsystem() {
    flywheelMotor = new SparkFlex(Constants.IntakeConstants.kFlywheelMotorId, MotorType.kBrushless);
  }

  /**
   * Run the intake to pull game pieces in.
   */
  public void intake() {
    flywheelMotor.set(INTAKE_SPEED);
  }

  /**
   * Run the intake in reverse to eject game pieces.
   */
  public void eject() {
    flywheelMotor.set(-INTAKE_SPEED);
  }

  /**
   * Stop the intake motor.
   */
  public void stop() {
    flywheelMotor.set(0);
  }

  /**
   * Command to run the intake while held.
   */
  public Command intakeCommand() {
    return this.runEnd(this::intake, this::stop).withName("Intake.Run");
  }

  /**
   * Command to eject while held.
   */
  public Command ejectCommand() {
    return this.runEnd(this::eject, this::stop).withName("Intake.Eject");
  }

  @Override
  public void periodic() {
    // Add any periodic logging here if needed
  }
}
