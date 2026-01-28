package frc.robot.subsystems.feeder;

import com.thethriftybot.ThriftyNova;

/**
 * Real hardware implementation of RollerIO using ThriftyNova.
 */
public class RollerIONova implements RollerIO {

    private final ThriftyNova nova;
    private double appliedDutyCycle = 0.0;

    /**
     * Creates a new RollerIONova.
     * 
     * @param canId the CAN ID of the ThriftyNova
     * @param inverted whether the motor is inverted
     * @param brakeMode whether to use brake mode (false = coast)
     * @param currentLimit the current limit in amps (currently unused)
     */
    public RollerIONova(int canId, boolean inverted, boolean brakeMode, int currentLimit) {
        nova = new ThriftyNova(canId);
        nova.setInverted(inverted);
        nova.setBrakeMode(brakeMode);
        // Note: Current limiting is handled through the controller's configuration
        // The raw ThriftyNova API doesn't expose setMaxCurrent directly
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        // ThriftyNova doesn't directly expose velocity/current in raw API
        // These are typically accessed through the YAMS wrapper
        inputs.velocityRPM = 0; // Would need encoder feedback
        inputs.currentAmps = 0; // Would need current sensing
        inputs.appliedVolts = appliedDutyCycle * 12.0; // Approximate
        inputs.tempCelsius = 25.0; // Nova doesn't report temp
        inputs.dutyCycle = appliedDutyCycle;
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
