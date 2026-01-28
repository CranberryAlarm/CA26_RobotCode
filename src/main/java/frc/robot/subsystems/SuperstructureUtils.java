package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Utility class for Superstructure tolerance and readiness calculations.
 * Extracted for testability and reuse.
 */
public final class SuperstructureUtils {

    // Tolerance constants matching Superstructure.java
    public static final AngularVelocity SHOOTER_TOLERANCE = RPM.of(100);
    public static final Angle TURRET_TOLERANCE = Degrees.of(1);
    public static final Angle HOOD_TOLERANCE = Degrees.of(2);

    private SuperstructureUtils() {
        // Utility class - no instantiation
    }

    /**
     * Checks if the shooter is at the target speed within tolerance.
     *
     * @param currentSpeed current shooter speed
     * @param targetSpeed  target shooter speed
     * @return true if within tolerance
     */
    public static boolean isShooterAtSpeed(AngularVelocity currentSpeed, AngularVelocity targetSpeed) {
        return Math.abs(currentSpeed.in(RPM) - targetSpeed.in(RPM)) < SHOOTER_TOLERANCE.in(RPM);
    }

    /**
     * Checks if the shooter is at the target speed within a custom tolerance.
     *
     * @param currentSpeed current shooter speed
     * @param targetSpeed  target shooter speed
     * @param tolerance    custom tolerance
     * @return true if within tolerance
     */
    public static boolean isShooterAtSpeed(AngularVelocity currentSpeed, AngularVelocity targetSpeed,
            AngularVelocity tolerance) {
        return Math.abs(currentSpeed.in(RPM) - targetSpeed.in(RPM)) < tolerance.in(RPM);
    }

    /**
     * Checks if the turret is on target within tolerance.
     *
     * @param currentAngle current turret angle
     * @param targetAngle  target turret angle
     * @return true if within tolerance
     */
    public static boolean isTurretOnTarget(Angle currentAngle, Angle targetAngle) {
        return Math.abs(currentAngle.in(Degrees) - targetAngle.in(Degrees)) < TURRET_TOLERANCE.in(Degrees);
    }

    /**
     * Checks if the turret is on target within a custom tolerance.
     *
     * @param currentAngle current turret angle
     * @param targetAngle  target turret angle
     * @param tolerance    custom tolerance
     * @return true if within tolerance
     */
    public static boolean isTurretOnTarget(Angle currentAngle, Angle targetAngle, Angle tolerance) {
        return Math.abs(currentAngle.in(Degrees) - targetAngle.in(Degrees)) < tolerance.in(Degrees);
    }

    /**
     * Checks if the hood is on target within tolerance.
     *
     * @param currentAngle current hood angle
     * @param targetAngle  target hood angle
     * @return true if within tolerance
     */
    public static boolean isHoodOnTarget(Angle currentAngle, Angle targetAngle) {
        return Math.abs(currentAngle.in(Degrees) - targetAngle.in(Degrees)) < HOOD_TOLERANCE.in(Degrees);
    }

    /**
     * Checks if the hood is on target within a custom tolerance.
     *
     * @param currentAngle current hood angle
     * @param targetAngle  target hood angle
     * @param tolerance    custom tolerance
     * @return true if within tolerance
     */
    public static boolean isHoodOnTarget(Angle currentAngle, Angle targetAngle, Angle tolerance) {
        return Math.abs(currentAngle.in(Degrees) - targetAngle.in(Degrees)) < tolerance.in(Degrees);
    }

    /**
     * Checks if all mechanisms are ready to shoot.
     *
     * @param shooterCurrent current shooter speed
     * @param shooterTarget  target shooter speed
     * @param turretCurrent  current turret angle
     * @param turretTarget   target turret angle
     * @param hoodCurrent    current hood angle
     * @param hoodTarget     target hood angle
     * @return true if all mechanisms are ready
     */
    public static boolean isReadyToShoot(
            AngularVelocity shooterCurrent, AngularVelocity shooterTarget,
            Angle turretCurrent, Angle turretTarget,
            Angle hoodCurrent, Angle hoodTarget) {
        return isShooterAtSpeed(shooterCurrent, shooterTarget) &&
                isTurretOnTarget(turretCurrent, turretTarget) &&
                isHoodOnTarget(hoodCurrent, hoodTarget);
    }

    /**
     * Calculates the shooter speed error.
     *
     * @param currentSpeed current speed
     * @param targetSpeed  target speed
     * @return error in RPM (positive = too slow, negative = too fast)
     */
    public static double getShooterError(AngularVelocity currentSpeed, AngularVelocity targetSpeed) {
        return targetSpeed.in(RPM) - currentSpeed.in(RPM);
    }

    /**
     * Calculates the turret angle error.
     *
     * @param currentAngle current angle
     * @param targetAngle  target angle
     * @return error in degrees
     */
    public static double getTurretError(Angle currentAngle, Angle targetAngle) {
        return targetAngle.in(Degrees) - currentAngle.in(Degrees);
    }

    /**
     * Calculates the hood angle error.
     *
     * @param currentAngle current angle
     * @param targetAngle  target angle
     * @return error in degrees
     */
    public static double getHoodError(Angle currentAngle, Angle targetAngle) {
        return targetAngle.in(Degrees) - currentAngle.in(Degrees);
    }

    /**
     * Result record for readiness status with details.
     */
    public record ReadinessStatus(
            boolean shooterReady,
            boolean turretReady,
            boolean hoodReady,
            double shooterError,
            double turretError,
            double hoodError) {
        public boolean isReady() {
            return shooterReady && turretReady && hoodReady;
        }
    }

    /**
     * Gets detailed readiness status for all mechanisms.
     *
     * @param shooterCurrent current shooter speed
     * @param shooterTarget  target shooter speed
     * @param turretCurrent  current turret angle
     * @param turretTarget   target turret angle
     * @param hoodCurrent    current hood angle
     * @param hoodTarget     target hood angle
     * @return detailed readiness status
     */
    public static ReadinessStatus getReadinessStatus(
            AngularVelocity shooterCurrent, AngularVelocity shooterTarget,
            Angle turretCurrent, Angle turretTarget,
            Angle hoodCurrent, Angle hoodTarget) {
        return new ReadinessStatus(
                isShooterAtSpeed(shooterCurrent, shooterTarget),
                isTurretOnTarget(turretCurrent, turretTarget),
                isHoodOnTarget(hoodCurrent, hoodTarget),
                getShooterError(shooterCurrent, shooterTarget),
                getTurretError(turretCurrent, turretTarget),
                getHoodError(hoodCurrent, hoodTarget));
    }
}
