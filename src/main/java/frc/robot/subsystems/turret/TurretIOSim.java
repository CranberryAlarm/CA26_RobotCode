package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulation implementation of TurretIO.
 * Uses WPILib physics simulation for realistic behavior.
 */
public class TurretIOSim implements TurretIO {

  private static final double MAX_ANGLE_DEGREES = 90.0;
  private static final double POSITION_TOLERANCE_DEGREES = 1.0;
  private static final double GEAR_RATIO = 40.0;
  private static final double MOI = 0.05; // kg*m^2

  // Turret simulation - horizontal rotation, so no gravity
  private final SingleJointedArmSim turretSim = new SingleJointedArmSim(
      DCMotor.getNEO(1),
      GEAR_RATIO,
      MOI,
      0.17, // arm length (approx radius)
      Math.toRadians(-MAX_ANGLE_DEGREES - 5),
      Math.toRadians(MAX_ANGLE_DEGREES + 5),
      false, // No gravity for horizontal turret
      Math.toRadians(0) // Start at center
  );

  private final PIDController pidController = new PIDController(0.1, 0, 0.01);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 7.5, 0);

  private double appliedVolts = 0.0;
  private double setpointDegrees = 0.0;
  private boolean closedLoop = false;

  public TurretIOSim() {
    pidController.setTolerance(POSITION_TOLERANCE_DEGREES);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Run closed loop if active
    if (closedLoop) {
      double currentDegrees = Math.toDegrees(turretSim.getAngleRads());
      double pidOutput = pidController.calculate(currentDegrees, setpointDegrees);
      double ffOutput = feedforward.calculate(pidController.getSetpoint(), 0);
      appliedVolts = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
    }

    // Update simulation
    turretSim.setInputVoltage(appliedVolts);
    turretSim.update(0.02); // 20ms loop time

    inputs.positionDegrees = Math.toDegrees(turretSim.getAngleRads());
    inputs.velocityDegreesPerSec = Math.toDegrees(turretSim.getVelocityRadPerSec());
    inputs.currentAmps = turretSim.getCurrentDrawAmps();
    inputs.appliedVolts = appliedVolts;
    inputs.tempCelsius = 25.0; // Ambient temp in sim
    inputs.setpointDegrees = setpointDegrees;
    inputs.atSetpoint = Math.abs(inputs.positionDegrees - setpointDegrees) < POSITION_TOLERANCE_DEGREES;
    inputs.atPositiveLimit = inputs.positionDegrees >= MAX_ANGLE_DEGREES - 1;
    inputs.atNegativeLimit = inputs.positionDegrees <= -MAX_ANGLE_DEGREES + 1;
  }

  @Override
  public void setPosition(double angleDegrees) {
    setpointDegrees = MathUtil.clamp(angleDegrees, -MAX_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
    closedLoop = true;
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    closedLoop = false;
    appliedVolts = dutyCycle * 12.0;
    setpointDegrees = Math.toDegrees(turretSim.getAngleRads());
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0;
    setpointDegrees = Math.toDegrees(turretSim.getAngleRads());
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    setpointDegrees = Math.toDegrees(turretSim.getAngleRads());
  }

  @Override
  public void resetPosition() {
    // In sim, we can't easily reset the physics state, so just update setpoint
    setpointDegrees = 0;
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pidController.setPID(kP, kI, kD);
  }
}
