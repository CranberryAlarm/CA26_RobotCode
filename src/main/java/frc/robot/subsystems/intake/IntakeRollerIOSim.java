package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of IntakeRollerIO.
 */
public class IntakeRollerIOSim implements IntakeRollerIO {

    private final FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), 0.001, 1.0),
        DCMotor.getNeoVortex(1)
    );

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        flywheelSim.setInputVoltage(appliedVolts);
        flywheelSim.update(0.02);

        inputs.velocityRPM = flywheelSim.getAngularVelocityRPM();
        inputs.currentAmps = flywheelSim.getCurrentDrawAmps();
        inputs.appliedVolts = appliedVolts;
        inputs.tempCelsius = 25.0;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        appliedVolts = MathUtil.clamp(dutyCycle * 12.0, -12.0, 12.0);
    }

    @Override
    public void stop() {
        appliedVolts = 0;
    }
}
