package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/**
 * Utility class for shoot-on-the-move trajectory calculations.
 * Extracted from ShootOnTheMoveCommand for testability.
 * 
 * This class provides pure functions for calculating:
 * - Time of flight based on distance
 * - Required shooter speed based on distance
 * - Required hood angle based on distance
 * - Corrective vectors for moving shots
 * - Target heading calculations
 */
public final class ShootingCalculations {

    private ShootingCalculations() {
        // Utility class - no instantiation
    }

    // ========== Interpolation Tables ==========
    
    // meters -> seconds: Time of flight based on distance to target
    private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_BY_DISTANCE = 
        InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(1.0, 1.0),
            Map.entry(4.86, 1.5));

    // meters -> RPM: Shooter speed based on distance to target
    private static final InterpolatingDoubleTreeMap SHOOTING_SPEED_BY_DISTANCE = 
        InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(2.0, 2700.0),
            Map.entry(3.0, 3000.0),
            Map.entry(4.0, 3300.0),
            Map.entry(4.86, 3750.0));

    // meters -> degrees: Hood angle based on distance to target
    private static final InterpolatingDoubleTreeMap HOOD_ANGLE_BY_DISTANCE = 
        InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(1.0, 15.0),
            Map.entry(2.0, 30.0),
            Map.entry(3.0, 45.0));

    // ========== Core Calculation Methods ==========

    /**
     * Calculates the time of flight for a projectile based on distance.
     * Uses linear interpolation from empirical data.
     * 
     * @param distanceToTarget distance to target
     * @return estimated time of flight in seconds
     */
    public static double getFlightTime(Distance distanceToTarget) {
        return TIME_OF_FLIGHT_BY_DISTANCE.get(distanceToTarget.in(Meters));
    }

    /**
     * Calculates the time of flight for a projectile based on distance in meters.
     * 
     * @param distanceMeters distance to target in meters
     * @return estimated time of flight in seconds
     */
    public static double getFlightTimeMeters(double distanceMeters) {
        return TIME_OF_FLIGHT_BY_DISTANCE.get(distanceMeters);
    }

    /**
     * Calculates the required shooter speed based on distance.
     * 
     * @param distanceToTarget distance to target
     * @return required shooter angular velocity
     */
    public static AngularVelocity calculateRequiredShooterSpeed(Distance distanceToTarget) {
        return RPM.of(SHOOTING_SPEED_BY_DISTANCE.get(distanceToTarget.in(Meters)));
    }

    /**
     * Calculates the required shooter speed based on distance in meters.
     * 
     * @param distanceMeters distance to target in meters
     * @return required shooter speed in RPM
     */
    public static double calculateRequiredShooterSpeedRPM(double distanceMeters) {
        return SHOOTING_SPEED_BY_DISTANCE.get(distanceMeters);
    }

    /**
     * Calculates the required hood angle based on distance.
     * 
     * @param distanceToTarget distance to target
     * @return required hood angle
     */
    public static Angle calculateRequiredHoodAngle(Distance distanceToTarget) {
        return Degrees.of(HOOD_ANGLE_BY_DISTANCE.get(distanceToTarget.in(Meters)));
    }

    /**
     * Calculates the required hood angle based on distance in meters.
     * 
     * @param distanceMeters distance to target in meters
     * @return required hood angle in degrees
     */
    public static double calculateRequiredHoodAngleDegrees(double distanceMeters) {
        return HOOD_ANGLE_BY_DISTANCE.get(distanceMeters);
    }

    // ========== Movement Compensation Methods ==========

    /**
     * Calculates the corrective vector to compensate for robot movement.
     * This vector represents where to aim "ahead" of the target when moving.
     * 
     * @param fieldVelocity current robot velocity in field coordinates
     * @param timeOfFlight estimated time for projectile to reach target
     * @return corrective translation to apply to target position
     */
    public static Translation2d calculateCorrectiveVector(ChassisSpeeds fieldVelocity, double timeOfFlight) {
        // Calculate where the robot will be after time of flight
        double futureX = fieldVelocity.vxMetersPerSecond * timeOfFlight;
        double futureY = fieldVelocity.vyMetersPerSecond * timeOfFlight;
        
        // Return the negation - we aim "behind" our future position relative to target
        return new Translation2d(futureX, futureY).unaryMinus();
    }

    /**
     * Calculates the corrective vector as a 3D translation (with z=0).
     * Useful for applying directly to 3D target points.
     * 
     * @param fieldVelocity current robot velocity in field coordinates
     * @param timeOfFlight estimated time for projectile to reach target
     * @return corrective translation as Translation3d
     */
    public static Translation3d calculateCorrectiveVector3d(ChassisSpeeds fieldVelocity, double timeOfFlight) {
        Translation2d correction2d = calculateCorrectiveVector(fieldVelocity, timeOfFlight);
        return new Translation3d(correction2d.getX(), correction2d.getY(), 0);
    }

    // ========== Target Calculation Methods ==========

    /**
     * Projects a 3D point onto the ground plane (z=0).
     * 
     * @param point3d 3D point to project
     * @return 2D point on ground plane
     */
    public static Translation2d projectToGround(Translation3d point3d) {
        return new Translation2d(point3d.getX(), point3d.getY());
    }

    /**
     * Calculates the 2D distance between two 3D points when projected to ground.
     * 
     * @param point1 first point
     * @param point2 second point
     * @return ground-plane distance
     */
    public static Distance calculateGroundDistance(Translation3d point1, Translation3d point2) {
        Translation2d ground1 = projectToGround(point1);
        Translation2d ground2 = projectToGround(point2);
        return Meters.of(ground1.getDistance(ground2));
    }

    /**
     * Calculates the turret heading angle to aim at a target, relative to robot heading.
     * 
     * @param robotPosition robot's current 2D position on field
     * @param targetPosition target's 2D position on field
     * @param robotHeading robot's current heading
     * @return turret angle relative to robot (positive = left, negative = right)
     */
    public static Angle calculateTurretHeading(
            Translation2d robotPosition, 
            Translation2d targetPosition, 
            Rotation2d robotHeading) {
        
        // Vector from robot to target
        Translation2d vectorToTarget = robotPosition.minus(targetPosition);
        
        // Get the absolute angle to target, then adjust for robot heading
        Rotation2d absoluteAngle = vectorToTarget.getAngle();
        Rotation2d relativeAngle = absoluteAngle.rotateBy(robotHeading.unaryMinus());
        
        return relativeAngle.getMeasure();
    }

    /**
     * Calculates the corrected target position accounting for robot movement.
     * 
     * @param originalTarget original target position (2D)
     * @param fieldVelocity robot's field velocity
     * @param timeOfFlight time for projectile to reach target
     * @return corrected target position
     */
    public static Translation2d calculateCorrectedTarget(
            Translation2d originalTarget,
            ChassisSpeeds fieldVelocity,
            double timeOfFlight) {
        
        Translation2d correction = calculateCorrectiveVector(fieldVelocity, timeOfFlight);
        return originalTarget.plus(correction);
    }

    /**
     * Complete shoot-on-the-move calculation result.
     */
    public record ShootingParameters(
        AngularVelocity shooterSpeed,
        Angle turretAngle,
        Angle hoodAngle,
        Distance correctedDistance,
        double timeOfFlight
    ) {}

    /**
     * Calculates all shooting parameters for shoot-on-the-move.
     * This is the main entry point for comprehensive shot calculation.
     * 
     * @param shooterPosition current shooter position (3D)
     * @param targetPosition target position (3D)
     * @param robotPosition robot's 2D position on field
     * @param robotHeading robot's current heading
     * @param fieldVelocity robot's velocity in field coordinates
     * @return complete shooting parameters
     */
    public static ShootingParameters calculateShootingParameters(
            Translation3d shooterPosition,
            Translation3d targetPosition,
            Translation2d robotPosition,
            Rotation2d robotHeading,
            ChassisSpeeds fieldVelocity) {
        
        // Project to ground plane for 2D calculations
        Translation2d shooterOnGround = projectToGround(shooterPosition);
        Translation2d targetOnGround = projectToGround(targetPosition);
        
        // Initial distance calculation for time of flight
        double initialDistance = shooterOnGround.getDistance(targetOnGround);
        double timeOfFlight = getFlightTimeMeters(initialDistance);
        
        // Apply movement correction
        Translation2d correctedTarget = calculateCorrectedTarget(targetOnGround, fieldVelocity, timeOfFlight);
        
        // Calculate corrected distance and heading
        Translation2d vectorToTarget = robotPosition.minus(correctedTarget);
        double correctedDistanceMeters = vectorToTarget.getNorm();
        
        Angle turretAngle = calculateTurretHeading(robotPosition, correctedTarget, robotHeading);
        AngularVelocity shooterSpeed = RPM.of(calculateRequiredShooterSpeedRPM(correctedDistanceMeters));
        Angle hoodAngle = Degrees.of(calculateRequiredHoodAngleDegrees(correctedDistanceMeters));
        
        return new ShootingParameters(
            shooterSpeed,
            turretAngle,
            hoodAngle,
            Meters.of(correctedDistanceMeters),
            timeOfFlight
        );
    }
}
