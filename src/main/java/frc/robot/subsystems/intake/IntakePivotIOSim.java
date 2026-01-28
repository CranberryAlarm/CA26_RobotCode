package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulation implementation of IntakePivotIO.
 */
public class IntakePivotIOSim implements IntakePivotIO {

  private static final double MIN_ANGLE_DEGREES = 0.0;
  private static final double MAX_ANGLE_DEGREES = 150.0;
  private static final double POSITION_TOLERANCE_DEGREES = 5.0;
  private static final double GEAR_RATIO = 83.33;

  private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
      DCMotor.getNeoVortex(1),
      GEAR_RATIO,
      0.5, // MOI
      0.3, // arm length (1 foot)
      Math.toRadians(MIN_ANGLE_DEGREES - 5),
      Math.toRadians(MAX_ANGLE_DEGREES + 5),
      true, // Simulate gravity
      Math.toRadians(0));

  private final ProfiledPIDController pidController = new ProfiledPIDController(
      0.1, 0, 0,
      new TrapezoidProfile.Constraints(360, 360));

  private double appliedVolts = 0.0;
  private double setpointDegrees = 0.0;
  private boolean closedLoop = false;

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    if (closedLoop) {
      double currentDegrees = Math.toDegrees(pivotSim.getAngleRads());
      appliedVolts = MathUtil.clamp(
          pidController.calculate(currentDegrees, setpointDegrees),
          -12.0, 12.0);
    }

    pivotSim.setInputVoltage(appliedVolts);
    pivotSim.update(0.02);

    inputs.positionDegrees = Math.toDegrees(pivotSim.getAngleRads());
    inputs.velocityDegreesPerSec = Math.toDegrees(pivotSim.getVelocityRadPerSec());
    inputs.currentAmps = pivotSim.getCurrentDrawAmps();
    inputs.appliedVolts = appliedVolts;
    inputs.tempCelsius = 25.0;
    inputs.setpointDegrees = setpointDegrees;
    inputs.atSetpoint = Math.abs(inputs.positionDegrees - setpointDegrees) < POSITION_TOLERANCE_DEGREES;
  }

  @Override
  public void setPosition(double angleDegrees) {
    setpointDegrees = MathUtil.clamp(angleDegrees, MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
    closedLoop = true;
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    closedLoop = false;
    appliedVolts = dutyCycle * 12.0;
    setpointDegrees = Math.toDegrees(pivotSim.getAngleRads());
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0;
    setpointDegrees = Math.toDegrees(pivotSim.getAngleRads());
  }

  @Override
  public void resetPosition() {
    setpointDegrees = 0;
  }
}
