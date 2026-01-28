package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of ShooterIO.
 * Uses WPILib physics simulation for realistic behavior.
 */
public class ShooterIOSim implements ShooterIO {

    // Flywheel simulation - 2 NEO motors, 1:1 gearing, reasonable MOI for 4" wheels
    private final FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), 0.004, 1.0),
        DCMotor.getNEO(2)
    );

    private final PIDController pidController = new PIDController(0.00936, 0, 0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.191, 0.11858, 0.0);

    private double appliedVolts = 0.0;
    private double velocitySetpointRPM = 0.0;
    private boolean closedLoop = false;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Run closed loop if active
        if (closedLoop && velocitySetpointRPM != 0) {
            double currentRPM = flywheelSim.getAngularVelocityRPM();
            double pidOutput = pidController.calculate(currentRPM, velocitySetpointRPM);
            double ffOutput = feedforward.calculate(velocitySetpointRPM / 60.0); // Convert to RPS for FF
            appliedVolts = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
        }

        // Update simulation
        flywheelSim.setInputVoltage(appliedVolts);
        flywheelSim.update(0.02); // 20ms loop time

        // Both motors run at same speed in simulation
        double velocityRPM = flywheelSim.getAngularVelocityRPM();
        inputs.leaderVelocityRPM = velocityRPM;
        inputs.followerVelocityRPM = velocityRPM;
        
        // Simulate current draw (rough approximation)
        inputs.leaderCurrentAmps = Math.abs(flywheelSim.getCurrentDrawAmps() / 2.0);
        inputs.followerCurrentAmps = Math.abs(flywheelSim.getCurrentDrawAmps() / 2.0);
        
        inputs.leaderAppliedVolts = appliedVolts;
        inputs.leaderTempCelsius = 25.0; // Ambient temp in sim
        inputs.followerTempCelsius = 25.0;
        inputs.velocitySetpointRPM = velocitySetpointRPM;
        inputs.atSetpoint = Math.abs(velocityRPM - velocitySetpointRPM) < 100;
    }

    @Override
    public void setVelocity(double velocityRPM) {
        velocitySetpointRPM = velocityRPM;
        closedLoop = true;
        
        if (velocityRPM == 0) {
            stop();
        }
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        closedLoop = false;
        velocitySetpointRPM = 0;
        appliedVolts = dutyCycle * 12.0;
    }

    @Override
    public void stop() {
        closedLoop = false;
        velocitySetpointRPM = 0;
        appliedVolts = 0;
    }

    @Override
    public void setVoltage(double volts) {
        closedLoop = false;
        velocitySetpointRPM = 0;
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        pidController.setPID(kP, kI, kD);
    }
}
