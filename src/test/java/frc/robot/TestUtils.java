package frc.robot;

import edu.wpi.first.hal.HAL;

/**
 * Utility class for FRC unit tests.
 * Provides common setup and helper methods following WPILib testing best
 * practices.
 */
public final class TestUtils {

    /** Acceptable error for floating-point comparisons */
    public static final double DELTA = 1e-3;

    /** Larger acceptable error for angle comparisons (in degrees) */
    public static final double ANGLE_DELTA = 0.5;

    /** Acceptable error for RPM comparisons */
    public static final double RPM_DELTA = 10.0;

    /** Acceptable error for distance comparisons (in meters) */
    public static final double DISTANCE_DELTA = 0.01;

    private TestUtils() {
        // Utility class - no instantiation
    }

    /**
     * Initializes the HAL for testing. Should be called in @BeforeEach methods.
     * 
     * @throws RuntimeException if HAL initialization fails
     */
    public static void initializeHAL() {
        if (!HAL.initialize(500, 0)) {
            throw new RuntimeException("Failed to initialize HAL");
        }
    }

    /**
     * Creates a tolerance check for angles in degrees.
     * 
     * @param expected  expected angle in degrees
     * @param actual    actual angle in degrees
     * @param tolerance tolerance in degrees
     * @return true if within tolerance
     */
    public static boolean isWithinAngleTolerance(double expected, double actual, double tolerance) {
        // Handle angle wrapping for values near +/-180
        double diff = Math.abs(expected - actual);
        diff = Math.min(diff, 360 - diff);
        return diff <= tolerance;
    }

    /**
     * Calculates distance between two 2D points.
     * 
     * @param x1 first point x
     * @param y1 first point y
     * @param x2 second point x
     * @param y2 second point y
     * @return Euclidean distance
     */
    public static double distance2D(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }
}
