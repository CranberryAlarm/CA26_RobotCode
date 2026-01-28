package frc.robot.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of RollerIO.
 */
public class RollerIOSim implements RollerIO {

  private final FlywheelSim flywheelSim;
  private double appliedVolts = 0.0;
  private double appliedDutyCycle = 0.0;

  /**
   * Creates a new RollerIOSim.
   *
   * @param gearRatio the gear ratio of the mechanism
   */
  public RollerIOSim(double gearRatio) {
    flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.001, gearRatio),
        DCMotor.getNEO(1));
  }

  /**
   * Creates a new RollerIOSim with default 4:1 gearing.
   */
  public RollerIOSim() {
    this(4.0);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    flywheelSim.setInputVoltage(appliedVolts);
    flywheelSim.update(0.02);

    inputs.velocityRPM = flywheelSim.getAngularVelocityRPM();
    inputs.currentAmps = flywheelSim.getCurrentDrawAmps();
    inputs.appliedVolts = appliedVolts;
    inputs.tempCelsius = 25.0;
    inputs.dutyCycle = appliedDutyCycle;
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    appliedDutyCycle = dutyCycle;
    appliedVolts = MathUtil.clamp(dutyCycle * 12.0, -12.0, 12.0);
  }

  @Override
  public void stop() {
    appliedDutyCycle = 0;
    appliedVolts = 0;
  }
}
