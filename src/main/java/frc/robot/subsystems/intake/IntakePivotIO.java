package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the intake pivot subsystem.
 * Abstracts hardware interactions for logging and replay support.
 */
public interface IntakePivotIO {

    /**
     * Input data for the intake pivot.
     */
    @AutoLog
    public static class IntakePivotIOInputs {
        /** Current pivot angle in degrees */
        public double positionDegrees = 0.0;
        
        /** Current pivot velocity in degrees per second */
        public double velocityDegreesPerSec = 0.0;
        
        /** Current draw in amps */
        public double currentAmps = 0.0;
        
        /** Applied voltage */
        public double appliedVolts = 0.0;
        
        /** Motor temperature in Celsius */
        public double tempCelsius = 0.0;
        
        /** Target position setpoint in degrees */
        public double setpointDegrees = 0.0;
        
        /** Whether the pivot is at the target position */
        public boolean atSetpoint = false;
    }

    /**
     * Updates all input values.
     * 
     * @param inputs the inputs object to update
     */
    default void updateInputs(IntakePivotIOInputs inputs) {}

    /**
     * Sets the pivot to a target angle.
     * 
     * @param angleDegrees the target angle in degrees
     */
    default void setPosition(double angleDegrees) {}

    /**
     * Sets the pivot motor to a duty cycle.
     * 
     * @param dutyCycle the duty cycle [-1, 1]
     */
    default void setDutyCycle(double dutyCycle) {}

    /**
     * Stops the pivot motor.
     */
    default void stop() {}

    /**
     * Resets the encoder position to zero.
     */
    default void resetPosition() {}
}
