package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Intake subsystem using AdvantageKit IO layer pattern.
 * Controls the intake roller and pivot mechanism.
 */
public class Intake extends SubsystemBase {

    private static final double INTAKE_SPEED = 1.0;

    private final IntakeRollerIO rollerIO;
    private final IntakePivotIO pivotIO;
    private final IntakeRollerIOInputsAutoLogged rollerInputs = new IntakeRollerIOInputsAutoLogged();
    private final IntakePivotIOInputsAutoLogged pivotInputs = new IntakePivotIOInputsAutoLogged();

    /**
     * Creates a new Intake subsystem.
     * 
     * @param rollerIO the roller IO implementation
     * @param pivotIO the pivot IO implementation
     */
    public Intake(IntakeRollerIO rollerIO, IntakePivotIO pivotIO) {
        this.rollerIO = rollerIO;
        this.pivotIO = pivotIO;
    }

    @Override
    public void periodic() {
        // Update and log inputs
        rollerIO.updateInputs(rollerInputs);
        pivotIO.updateInputs(pivotInputs);
        Logger.processInputs("Intake/Roller", rollerInputs);
        Logger.processInputs("Intake/Pivot", pivotInputs);
    }

    /**
     * Command to run the intake while held.
     * 
     * @return a command that runs the intake
     */
    public Command intakeCommand() {
        return Commands.startEnd(
            () -> rollerIO.setDutyCycle(INTAKE_SPEED),
            () -> rollerIO.stop(),
            this
        ).withName("Intake.Run");
    }

    /**
     * Command to eject while held.
     * 
     * @return a command that ejects
     */
    public Command ejectCommand() {
        return Commands.startEnd(
            () -> rollerIO.setDutyCycle(-INTAKE_SPEED),
            () -> rollerIO.stop(),
            this
        ).withName("Intake.Eject");
    }

    /**
     * Command to set the pivot angle.
     * 
     * @param angle the target angle
     * @return a command that sets the pivot angle
     */
    public Command setPivotAngle(Angle angle) {
        return Commands.runOnce(() -> pivotIO.setPosition(angle.in(Degrees)), this)
            .withName("IntakePivot.SetAngle");
    }

    /**
     * Command to deploy the intake (pivot out).
     * 
     * @return a command that deploys the intake
     */
    public Command deploy() {
        return setPivotAngle(Degrees.of(120));
    }

    /**
     * Command to stow the intake (pivot in).
     * 
     * @return a command that stows the intake
     */
    public Command stow() {
        return setPivotAngle(Degrees.of(0));
    }

    /**
     * Command to reset the pivot encoder.
     * 
     * @return a command that resets the encoder
     */
    public Command rezero() {
        return Commands.runOnce(() -> pivotIO.resetPosition(), this)
            .withName("IntakePivot.Rezero");
    }

    /**
     * Gets the current pivot angle.
     * 
     * @return the current pivot angle
     */
    public Angle getPivotAngle() {
        return Degrees.of(pivotInputs.positionDegrees);
    }

    /**
     * Checks if the pivot is at its setpoint.
     * 
     * @return true if at setpoint
     */
    public boolean pivotAtSetpoint() {
        return pivotInputs.atSetpoint;
    }

    /**
     * Command to deploy intake and run roller while held.
     * Stops roller when released.
     * 
     * @return a command that deploys and rolls
     */
    public Command deployAndRollCommand() {
        return Commands.run(() -> {
            pivotIO.setPosition(148); // deployed
            rollerIO.setDutyCycle(INTAKE_SPEED);
        }, this).finallyDo(() -> {
            rollerIO.stop();
            pivotIO.setPosition(115); // hold
        }).withName("Intake.DeployAndRoll");
    }

    /**
     * Command to back feed and roll while held.
     * 
     * @return a command that back feeds
     */
    public Command backFeedAndRollCommand() {
        return Commands.run(() -> {
            pivotIO.setPosition(148); // deployed
            // rollerIO.setDutyCycle(-INTAKE_SPEED);
        }, this).finallyDo(() -> {
            rollerIO.stop();
            pivotIO.setPosition(115); // hold
        }).withName("Intake.BackFeedAndRoll");
    }
}
