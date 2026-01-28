package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the intake roller subsystem.
 * Abstracts hardware interactions for logging and replay support.
 */
public interface IntakeRollerIO {

    /**
     * Input data for the intake roller.
     */
    @AutoLog
    public static class IntakeRollerIOInputs {
        /** Current roller velocity in RPM */
        public double velocityRPM = 0.0;
        
        /** Current draw in amps */
        public double currentAmps = 0.0;
        
        /** Applied voltage */
        public double appliedVolts = 0.0;
        
        /** Motor temperature in Celsius */
        public double tempCelsius = 0.0;
    }

    /**
     * Updates all input values.
     * 
     * @param inputs the inputs object to update
     */
    default void updateInputs(IntakeRollerIOInputs inputs) {}

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
