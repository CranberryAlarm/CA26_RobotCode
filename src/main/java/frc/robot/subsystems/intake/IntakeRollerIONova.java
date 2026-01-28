package frc.robot.subsystems.intake;

import com.thethriftybot.ThriftyNova;

import frc.robot.Constants;

/**
 * Real hardware implementation of IntakeRollerIO using ThriftyNova.
 */
public class IntakeRollerIONova implements IntakeRollerIO {

    private final ThriftyNova nova;
    private double appliedDutyCycle = 0.0;

    public IntakeRollerIONova() {
        nova = new ThriftyNova(Constants.IntakeConstants.kRollerMotorId);
        nova.setInverted(true);
        nova.setBrakeMode(false); // Coast mode
        // Note: Current limiting is handled through the controller's configuration
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        // ThriftyNova doesn't directly expose velocity/current in raw API
        inputs.velocityRPM = 0; // Would need encoder feedback
        inputs.currentAmps = 0; // Would need current sensing
        inputs.appliedVolts = appliedDutyCycle * 12.0; // Approximate
        inputs.tempCelsius = 25.0; // Nova doesn't report temp
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        appliedDutyCycle = dutyCycle;
        nova.setPercent(dutyCycle);
    }

    @Override
    public void stop() {
        appliedDutyCycle = 0;
        nova.setPercent(0);
    }
}
