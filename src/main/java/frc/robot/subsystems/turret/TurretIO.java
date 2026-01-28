package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the turret subsystem.
 * Abstracts hardware interactions for logging and replay support.
 */
public interface TurretIO {

    /**
     * Input data for the turret subsystem.
     */
    @AutoLog
    public static class TurretIOInputs {
        /** Current turret angle in degrees */
        public double positionDegrees = 0.0;
        
        /** Current turret velocity in degrees per second */
        public double velocityDegreesPerSec = 0.0;
        
        /** Current draw in amps */
        public double currentAmps = 0.0;
        
        /** Applied voltage */
        public double appliedVolts = 0.0;
        
        /** Motor temperature in Celsius */
        public double tempCelsius = 0.0;
        
        /** Target position setpoint in degrees */
        public double setpointDegrees = 0.0;
        
        /** Whether the turret is at the target position */
        public boolean atSetpoint = false;
        
        /** Whether the turret is at the positive limit */
        public boolean atPositiveLimit = false;
        
        /** Whether the turret is at the negative limit */
        public boolean atNegativeLimit = false;
    }

    /**
     * Updates all input values. Should be called once per cycle.
     * 
     * @param inputs the inputs object to update
     */
    default void updateInputs(TurretIOInputs inputs) {}

    /**
     * Sets the turret to a target angle using closed-loop control.
     * 
     * @param angleDegrees the target angle in degrees
     */
    default void setPosition(double angleDegrees) {}

    /**
     * Sets the turret motor to a duty cycle (open loop).
     * 
     * @param dutyCycle the duty cycle [-1, 1]
     */
    default void setDutyCycle(double dutyCycle) {}

    /**
     * Stops the turret motor.
     */
    default void stop() {}

    /**
     * Sets voltage directly (for SysId).
     * 
     * @param volts the voltage to apply
     */
    default void setVoltage(double volts) {}

    /**
     * Resets the encoder position to zero.
     */
    default void resetPosition() {}

    /**
     * Configures the closed-loop PID gains.
     * 
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     */
    default void configurePID(double kP, double kI, double kD) {}
}
