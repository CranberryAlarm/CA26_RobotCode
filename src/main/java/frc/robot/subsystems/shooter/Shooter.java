package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Shooter subsystem using AdvantageKit IO layer pattern.
 * Controls a dual-motor flywheel shooter.
 */
public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  /**
   * Creates a new Shooter subsystem.
   *
   * @param io the IO implementation to use
   */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update and log inputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  /**
   * Command to set the shooter to a specific velocity.
   *
   * @param speed the target velocity
   * @return a command that sets the shooter speed
   */
  public Command setSpeed(AngularVelocity speed) {
    return Commands.runOnce(() -> io.setVelocity(speed.in(RPM)), this)
        .withName("Shooter.SetSpeed");
  }

  /**
   * Command to dynamically set the shooter velocity.
   *
   * @param speedSupplier supplier for the target velocity
   * @return a command that continuously updates the shooter speed
   */
  public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier) {
    return Commands.run(() -> io.setVelocity(speedSupplier.get().in(RPM)), this)
        .withName("Shooter.SetSpeedDynamic");
  }

  /**
   * Command to spin up the shooter to the default shooting speed.
   *
   * @return a command that spins up the shooter
   */
  public Command spinUp() {
    return setSpeed(RPM.of(5500));
  }

  /**
   * Command to stop the shooter.
   *
   * @return a command that stops the shooter
   */
  public Command stop() {
    return Commands.runOnce(() -> io.stop(), this)
        .withName("Shooter.Stop");
  }

  /**
   * Gets the current shooter velocity.
   *
   * @return the current velocity
   */
  public AngularVelocity getSpeed() {
    return RPM.of(inputs.leaderVelocityRPM);
  }

  /**
   * Checks if the shooter is at the target speed.
   *
   * @return true if at setpoint
   */
  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }

  /**
   * Gets the current velocity setpoint.
   *
   * @return the setpoint in RPM
   */
  public double getSetpointRPM() {
    return inputs.velocitySetpointRPM;
  }

  /**
   * Gets the wheel radius for tangential velocity calculation.
   *
   * @return the wheel radius
   */
  private Distance wheelRadius() {
    return Inches.of(4).div(2);
  }

  /**
   * Calculates the tangential velocity at the edge of the shooter wheel.
   *
   * @return the tangential velocity
   */
  public LinearVelocity getTangentialVelocity() {
    return MetersPerSecond.of(getSpeed().in(RadiansPerSecond)
        * wheelRadius().in(Meters));
  }
}
