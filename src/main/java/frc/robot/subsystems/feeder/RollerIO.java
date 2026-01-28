package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

/**
 * Generic IO interface for roller-style mechanisms (Hopper, Kicker).
 * Abstracts hardware interactions for logging and replay support.
 */
public interface RollerIO {

    /**
     * Input data for a roller mechanism.
     */
    @AutoLog
    public static class RollerIOInputs {
        /** Current roller velocity in RPM */
        public double velocityRPM = 0.0;
        
        /** Current draw in amps */
        public double currentAmps = 0.0;
        
        /** Applied voltage */
        public double appliedVolts = 0.0;
        
        /** Motor temperature in Celsius */
        public double tempCelsius = 0.0;
        
        /** Current duty cycle being applied */
        public double dutyCycle = 0.0;
    }

    /**
     * Updates all input values.
     * 
     * @param inputs the inputs object to update
     */
    default void updateInputs(RollerIOInputs inputs) {}

    /**
     * Sets the roller motor to a duty cycle.
     * 
     * @param dutyCycle the duty cycle [-1, 1]
     */
    default void setDutyCycle(double dutyCycle) {}

    /**
     * Stops the roller motor.
     */
    default void stop() {}
}
