package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulation implementation of HoodIO.
 * Uses WPILib physics simulation for realistic behavior.
 */
public class HoodIOSim implements HoodIO {

  private static final double MIN_ANGLE_DEGREES = 0.0;
  private static final double MAX_ANGLE_DEGREES = 90.0;
  private static final double POSITION_TOLERANCE_DEGREES = 2.0;
  private static final double GEAR_RATIO = 50.0;
  private static final double MOI = 0.001; // kg*m^2

  // Hood simulation - vertical pivot with gravity
  private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(
      DCMotor.getNEO(1),
      GEAR_RATIO,
      MOI,
      0.1, // hood length
      Math.toRadians(MIN_ANGLE_DEGREES - 5),
      Math.toRadians(MAX_ANGLE_DEGREES + 5),
      true, // Simulate gravity
      Math.toRadians(0) // Start at stowed position
  );

  private final ProfiledPIDController pidController = new ProfiledPIDController(
      0.1, 0, 0.01,
      new TrapezoidProfile.Constraints(90, 90) // deg/s, deg/s^2
  );
  private final ArmFeedforward feedforward = new ArmFeedforward(0, 0.3, 0.1);

  private double appliedVolts = 0.0;
  private double setpointDegrees = 0.0;
  private boolean closedLoop = false;

  public HoodIOSim() {
    pidController.setTolerance(POSITION_TOLERANCE_DEGREES);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Run closed loop if active
    if (closedLoop) {
      double currentRadians = hoodSim.getAngleRads();
      double pidOutput = pidController.calculate(Math.toDegrees(currentRadians), setpointDegrees);
      double ffOutput = feedforward.calculate(
          Math.toRadians(pidController.getSetpoint().position),
          pidController.getSetpoint().velocity);
      appliedVolts = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
    }

    // Update simulation
    hoodSim.setInputVoltage(appliedVolts);
    hoodSim.update(0.02); // 20ms loop time

    inputs.positionDegrees = Math.toDegrees(hoodSim.getAngleRads());
    inputs.velocityDegreesPerSec = Math.toDegrees(hoodSim.getVelocityRadPerSec());
    inputs.currentAmps = hoodSim.getCurrentDrawAmps();
    inputs.appliedVolts = appliedVolts;
    inputs.tempCelsius = 25.0; // Ambient temp in sim
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
    setpointDegrees = Math.toDegrees(hoodSim.getAngleRads());
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0;
    setpointDegrees = Math.toDegrees(hoodSim.getAngleRads());
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    setpointDegrees = Math.toDegrees(hoodSim.getAngleRads());
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pidController.setPID(kP, kI, kD);
  }
}
