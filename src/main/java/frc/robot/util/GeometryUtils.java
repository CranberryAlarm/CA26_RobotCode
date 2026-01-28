package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Utility class for geometry calculations commonly used in FRC.
 * Provides pure functions for pose transformations and calculations.
 */
public final class GeometryUtils {

    private GeometryUtils() {
        // Utility class - no instantiation
    }

    /**
     * Calculates the distance between two poses.
     * 
     * @param pose1 first pose
     * @param pose2 second pose
     * @return distance in meters
     */
    public static double distanceBetween(Pose2d pose1, Pose2d pose2) {
        return pose1.getTranslation().getDistance(pose2.getTranslation());
    }

    /**
     * Calculates the distance between two translation2ds.
     * 
     * @param t1 first translation
     * @param t2 second translation
     * @return distance in meters
     */
    public static double distanceBetween(Translation2d t1, Translation2d t2) {
        return t1.getDistance(t2);
    }

    /**
     * Calculates the 3D distance between two Translation3d points.
     * 
     * @param t1 first point
     * @param t2 second point
     * @return 3D Euclidean distance
     */
    public static double distanceBetween3D(Translation3d t1, Translation3d t2) {
        double dx = t2.getX() - t1.getX();
        double dy = t2.getY() - t1.getY();
        double dz = t2.getZ() - t1.getZ();
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    /**
     * Calculates the angle from one point to another.
     * 
     * @param from starting point
     * @param to ending point
     * @return angle from first point to second
     */
    public static Rotation2d angleTo(Translation2d from, Translation2d to) {
        return to.minus(from).getAngle();
    }

    /**
     * Calculates the relative angle to a target from a robot's perspective.
     * 
     * @param robotPose robot's current pose
     * @param targetPosition target position
     * @return angle to target relative to robot's heading
     */
    public static Rotation2d relativeAngleTo(Pose2d robotPose, Translation2d targetPosition) {
        Rotation2d absoluteAngle = angleTo(robotPose.getTranslation(), targetPosition);
        return absoluteAngle.minus(robotPose.getRotation());
    }

    /**
     * Checks if a point is within a specified radius of another point.
     * 
     * @param center center point
     * @param point point to check
     * @param radius radius in meters
     * @return true if within radius
     */
    public static boolean isWithinRadius(Translation2d center, Translation2d point, double radius) {
        return center.getDistance(point) <= radius;
    }

    /**
     * Checks if a robot is facing a target within a tolerance angle.
     * 
     * @param robotPose robot's pose
     * @param targetPosition target position
     * @param toleranceRadians tolerance in radians
     * @return true if robot is facing target within tolerance
     */
    public static boolean isFacingTarget(Pose2d robotPose, Translation2d targetPosition, double toleranceRadians) {
        Rotation2d relativeAngle = relativeAngleTo(robotPose, targetPosition);
        return Math.abs(relativeAngle.getRadians()) <= toleranceRadians;
    }

    /**
     * Interpolates between two translations.
     * 
     * @param start starting point
     * @param end ending point
     * @param t interpolation factor (0 = start, 1 = end)
     * @return interpolated point
     */
    public static Translation2d lerp(Translation2d start, Translation2d end, double t) {
        t = Math.max(0, Math.min(1, t)); // Clamp t to [0, 1]
        return start.interpolate(end, t);
    }

    /**
     * Mirrors a pose for the opposite alliance.
     * Assumes a field width of approximately 16.54 meters.
     * 
     * @param pose pose to mirror
     * @param fieldLength length of the field in meters
     * @return mirrored pose
     */
    public static Pose2d mirrorPose(Pose2d pose, double fieldLength) {
        return new Pose2d(
            fieldLength - pose.getX(),
            pose.getY(),
            new Rotation2d(Math.PI).minus(pose.getRotation())
        );
    }

    /**
     * Converts a translation to polar coordinates.
     * 
     * @param translation the translation to convert
     * @return array of [radius, angle in radians]
     */
    public static double[] toPolar(Translation2d translation) {
        double radius = translation.getNorm();
        double angle = Math.atan2(translation.getY(), translation.getX());
        return new double[] { radius, angle };
    }

    /**
     * Creates a translation from polar coordinates.
     * 
     * @param radius distance from origin
     * @param angleRadians angle in radians
     * @return Translation2d from polar coordinates
     */
    public static Translation2d fromPolar(double radius, double angleRadians) {
        return new Translation2d(
            radius * Math.cos(angleRadians),
            radius * Math.sin(angleRadians)
        );
    }

    /**
     * Normalizes an angle to be within [-pi, pi].
     * 
     * @param angleRadians angle in radians
     * @return normalized angle
     */
    public static double normalizeAngle(double angleRadians) {
        double result = angleRadians % (2 * Math.PI);
        if (result > Math.PI) {
            result -= 2 * Math.PI;
        } else if (result < -Math.PI) {
            result += 2 * Math.PI;
        }
        return result;
    }

    /**
     * Calculates the shortest angular difference between two angles.
     * 
     * @param fromRadians starting angle in radians
     * @param toRadians ending angle in radians
     * @return shortest angular difference (can be positive or negative)
     */
    public static double shortestAngularDifference(double fromRadians, double toRadians) {
        double diff = normalizeAngle(toRadians - fromRadians);
        return diff;
    }
}
