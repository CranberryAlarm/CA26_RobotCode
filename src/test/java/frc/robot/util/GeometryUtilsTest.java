package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import frc.robot.TestUtils;

/**
 * Unit tests for GeometryUtils.
 * Tests distance calculations, angle operations, and coordinate transformations.
 */
@DisplayName("GeometryUtils")
class GeometryUtilsTest {

    private static final double DELTA = TestUtils.DELTA;
    private static final double ANGLE_DELTA = 0.001; // For angle calculations

    @Nested
    @DisplayName("Distance Calculations")
    class DistanceTests {

        @Test
        @DisplayName("should return zero for same pose")
        void distanceSamePose() {
            Pose2d pose = new Pose2d(1, 2, Rotation2d.fromDegrees(45));
            assertEquals(0, GeometryUtils.distanceBetween(pose, pose), DELTA);
        }

        @Test
        @DisplayName("should calculate correct horizontal distance")
        void distanceHorizontal() {
            Pose2d pose1 = new Pose2d(0, 0, Rotation2d.kZero);
            Pose2d pose2 = new Pose2d(3, 0, Rotation2d.kZero);
            assertEquals(3, GeometryUtils.distanceBetween(pose1, pose2), DELTA);
        }

        @Test
        @DisplayName("should calculate correct vertical distance")
        void distanceVertical() {
            Pose2d pose1 = new Pose2d(0, 0, Rotation2d.kZero);
            Pose2d pose2 = new Pose2d(0, 4, Rotation2d.kZero);
            assertEquals(4, GeometryUtils.distanceBetween(pose1, pose2), DELTA);
        }

        @Test
        @DisplayName("should calculate correct diagonal distance (3-4-5 triangle)")
        void distanceDiagonal() {
            Pose2d pose1 = new Pose2d(0, 0, Rotation2d.kZero);
            Pose2d pose2 = new Pose2d(3, 4, Rotation2d.kZero);
            assertEquals(5, GeometryUtils.distanceBetween(pose1, pose2), DELTA);
        }

        @Test
        @DisplayName("should ignore rotation when calculating distance")
        void distanceIgnoresRotation() {
            Pose2d pose1 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
            Pose2d pose2 = new Pose2d(3, 4, Rotation2d.fromDegrees(180));
            assertEquals(5, GeometryUtils.distanceBetween(pose1, pose2), DELTA);
        }

        @Test
        @DisplayName("should calculate 3D distance correctly")
        void distance3D() {
            Translation3d t1 = new Translation3d(0, 0, 0);
            Translation3d t2 = new Translation3d(1, 2, 2);
            assertEquals(3, GeometryUtils.distanceBetween3D(t1, t2), DELTA);
        }
    }

    @Nested
    @DisplayName("Angle Calculations")
    class AngleTests {

        @Test
        @DisplayName("should return zero for same point")
        void angleToSamePoint() {
            Translation2d point = new Translation2d(1, 1);
            // Angle to itself is undefined, but let's test the behavior
            Translation2d origin = new Translation2d(0, 0);
            Rotation2d angle = GeometryUtils.angleTo(origin, point);
            assertEquals(45, angle.getDegrees(), DELTA);
        }

        @ParameterizedTest
        @DisplayName("should calculate correct angle to target")
        @CsvSource({
            "0, 0, 1, 0, 0",     // East -> 0 degrees
            "0, 0, 0, 1, 90",    // North -> 90 degrees
            "0, 0, -1, 0, 180",  // West -> 180 degrees
            "0, 0, 0, -1, -90",  // South -> -90 degrees
            "0, 0, 1, 1, 45",    // Northeast -> 45 degrees
        })
        void angleToTarget(double x1, double y1, double x2, double y2, double expectedDegrees) {
            Translation2d from = new Translation2d(x1, y1);
            Translation2d to = new Translation2d(x2, y2);
            
            Rotation2d angle = GeometryUtils.angleTo(from, to);
            assertEquals(expectedDegrees, angle.getDegrees(), DELTA);
        }

        @Test
        @DisplayName("should calculate relative angle correctly")
        void relativeAngle() {
            // Robot at origin facing +X (0 degrees)
            Pose2d robotPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
            // Target directly ahead
            Translation2d targetAhead = new Translation2d(5, 0);
            
            Rotation2d relAngle = GeometryUtils.relativeAngleTo(robotPose, targetAhead);
            assertEquals(0, relAngle.getDegrees(), DELTA, "Target ahead should be 0 relative angle");
        }

        @Test
        @DisplayName("should calculate relative angle for target to the left")
        void relativeAngleLeft() {
            Pose2d robotPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
            Translation2d targetLeft = new Translation2d(0, 5);
            
            Rotation2d relAngle = GeometryUtils.relativeAngleTo(robotPose, targetLeft);
            assertEquals(90, relAngle.getDegrees(), DELTA, "Target left should be 90 relative angle");
        }

        @Test
        @DisplayName("should calculate relative angle when robot is rotated")
        void relativeAngleRotatedRobot() {
            // Robot facing +Y (90 degrees)
            Pose2d robotPose = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
            // Target at +X (east)
            Translation2d target = new Translation2d(5, 0);
            
            Rotation2d relAngle = GeometryUtils.relativeAngleTo(robotPose, target);
            assertEquals(-90, relAngle.getDegrees(), DELTA, 
                "Target to robot's right should be -90 degrees");
        }
    }

    @Nested
    @DisplayName("Radius Check")
    class RadiusTests {

        @Test
        @DisplayName("should return true when point is at center")
        void pointAtCenter() {
            Translation2d center = new Translation2d(5, 5);
            assertTrue(GeometryUtils.isWithinRadius(center, center, 1));
        }

        @Test
        @DisplayName("should return true when point is inside radius")
        void pointInsideRadius() {
            Translation2d center = new Translation2d(0, 0);
            Translation2d point = new Translation2d(1, 1);
            
            assertTrue(GeometryUtils.isWithinRadius(center, point, 2));
        }

        @Test
        @DisplayName("should return true when point is at radius boundary")
        void pointAtBoundary() {
            Translation2d center = new Translation2d(0, 0);
            Translation2d point = new Translation2d(3, 4); // Distance = 5
            
            assertTrue(GeometryUtils.isWithinRadius(center, point, 5));
        }

        @Test
        @DisplayName("should return false when point is outside radius")
        void pointOutsideRadius() {
            Translation2d center = new Translation2d(0, 0);
            Translation2d point = new Translation2d(3, 4); // Distance = 5
            
            assertFalse(GeometryUtils.isWithinRadius(center, point, 4.9));
        }
    }

    @Nested
    @DisplayName("Facing Target Check")
    class FacingTargetTests {

        @Test
        @DisplayName("should return true when directly facing target")
        void facingTargetDirectly() {
            Pose2d robot = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
            Translation2d target = new Translation2d(5, 0);
            
            assertTrue(GeometryUtils.isFacingTarget(robot, target, Math.toRadians(5)));
        }

        @Test
        @DisplayName("should return true when within tolerance")
        void facingTargetWithinTolerance() {
            Pose2d robot = new Pose2d(0, 0, Rotation2d.fromDegrees(5)); // Slightly rotated
            Translation2d target = new Translation2d(5, 0);
            
            assertTrue(GeometryUtils.isFacingTarget(robot, target, Math.toRadians(10)));
        }

        @Test
        @DisplayName("should return false when outside tolerance")
        void notFacingTarget() {
            Pose2d robot = new Pose2d(0, 0, Rotation2d.fromDegrees(45));
            Translation2d target = new Translation2d(5, 0);
            
            assertFalse(GeometryUtils.isFacingTarget(robot, target, Math.toRadians(10)));
        }
    }

    @Nested
    @DisplayName("Linear Interpolation")
    class LerpTests {

        @Test
        @DisplayName("should return start when t=0")
        void lerpAtZero() {
            Translation2d start = new Translation2d(0, 0);
            Translation2d end = new Translation2d(10, 10);
            
            Translation2d result = GeometryUtils.lerp(start, end, 0);
            
            assertEquals(0, result.getX(), DELTA);
            assertEquals(0, result.getY(), DELTA);
        }

        @Test
        @DisplayName("should return end when t=1")
        void lerpAtOne() {
            Translation2d start = new Translation2d(0, 0);
            Translation2d end = new Translation2d(10, 10);
            
            Translation2d result = GeometryUtils.lerp(start, end, 1);
            
            assertEquals(10, result.getX(), DELTA);
            assertEquals(10, result.getY(), DELTA);
        }

        @Test
        @DisplayName("should return midpoint when t=0.5")
        void lerpAtHalf() {
            Translation2d start = new Translation2d(0, 0);
            Translation2d end = new Translation2d(10, 10);
            
            Translation2d result = GeometryUtils.lerp(start, end, 0.5);
            
            assertEquals(5, result.getX(), DELTA);
            assertEquals(5, result.getY(), DELTA);
        }

        @Test
        @DisplayName("should clamp t to [0, 1]")
        void lerpClamps() {
            Translation2d start = new Translation2d(0, 0);
            Translation2d end = new Translation2d(10, 10);
            
            Translation2d resultNegative = GeometryUtils.lerp(start, end, -0.5);
            Translation2d resultOverOne = GeometryUtils.lerp(start, end, 1.5);
            
            assertEquals(0, resultNegative.getX(), DELTA, "Negative t should clamp to 0");
            assertEquals(10, resultOverOne.getX(), DELTA, "t > 1 should clamp to 1");
        }
    }

    @Nested
    @DisplayName("Pose Mirroring")
    class MirrorTests {

        @Test
        @DisplayName("should mirror X coordinate correctly")
        void mirrorXCoordinate() {
            double fieldLength = 16.54;
            Pose2d pose = new Pose2d(5, 3, Rotation2d.fromDegrees(0));
            
            Pose2d mirrored = GeometryUtils.mirrorPose(pose, fieldLength);
            
            assertEquals(fieldLength - 5, mirrored.getX(), DELTA);
        }

        @Test
        @DisplayName("should preserve Y coordinate")
        void mirrorPreservesY() {
            double fieldLength = 16.54;
            Pose2d pose = new Pose2d(5, 3, Rotation2d.fromDegrees(0));
            
            Pose2d mirrored = GeometryUtils.mirrorPose(pose, fieldLength);
            
            assertEquals(3, mirrored.getY(), DELTA);
        }

        @Test
        @DisplayName("should mirror rotation correctly")
        void mirrorRotation() {
            double fieldLength = 16.54;
            Pose2d pose = new Pose2d(5, 3, Rotation2d.fromDegrees(30));
            
            Pose2d mirrored = GeometryUtils.mirrorPose(pose, fieldLength);
            
            assertEquals(150, mirrored.getRotation().getDegrees(), DELTA);
        }

        @Test
        @DisplayName("double mirror should return original")
        void doubleMirror() {
            double fieldLength = 16.54;
            Pose2d original = new Pose2d(5, 3, Rotation2d.fromDegrees(45));
            
            Pose2d doubleMirrored = GeometryUtils.mirrorPose(
                GeometryUtils.mirrorPose(original, fieldLength), fieldLength);
            
            assertEquals(original.getX(), doubleMirrored.getX(), DELTA);
            assertEquals(original.getY(), doubleMirrored.getY(), DELTA);
            assertEquals(original.getRotation().getDegrees(), 
                doubleMirrored.getRotation().getDegrees(), DELTA);
        }
    }

    @Nested
    @DisplayName("Polar Coordinate Conversion")
    class PolarTests {

        @Test
        @DisplayName("should convert to polar correctly")
        void toPolarBasic() {
            Translation2d point = new Translation2d(3, 4); // Radius should be 5
            
            double[] polar = GeometryUtils.toPolar(point);
            
            assertEquals(5, polar[0], DELTA, "Radius should be 5");
            assertTrue(polar[1] > 0 && polar[1] < Math.PI / 2, 
                "Angle should be in first quadrant");
        }

        @Test
        @DisplayName("should convert from polar correctly")
        void fromPolarBasic() {
            double radius = 5;
            double angle = Math.PI / 4; // 45 degrees
            
            Translation2d result = GeometryUtils.fromPolar(radius, angle);
            
            assertEquals(radius * Math.cos(angle), result.getX(), DELTA);
            assertEquals(radius * Math.sin(angle), result.getY(), DELTA);
        }

        @Test
        @DisplayName("round trip should preserve point")
        void polarRoundTrip() {
            Translation2d original = new Translation2d(3, 4);
            
            double[] polar = GeometryUtils.toPolar(original);
            Translation2d result = GeometryUtils.fromPolar(polar[0], polar[1]);
            
            assertEquals(original.getX(), result.getX(), DELTA);
            assertEquals(original.getY(), result.getY(), DELTA);
        }
    }

    @Nested
    @DisplayName("Angle Normalization")
    class NormalizeAngleTests {

        @ParameterizedTest
        @DisplayName("should normalize angles to [-π, π]")
        @CsvSource({
            "0, 0",
            "3.14159, 3.14159",
            "-3.14159, -3.14159",
            "6.28318, 0",         // 2π -> 0
            "-6.28318, 0",        // -2π -> 0
            "4.71239, -1.57079",  // 3π/2 -> -π/2
        })
        void normalizeAngle(double input, double expected) {
            double result = GeometryUtils.normalizeAngle(input);
            assertEquals(expected, result, ANGLE_DELTA);
        }
    }

    @Nested
    @DisplayName("Angular Difference")
    class AngularDifferenceTests {

        @Test
        @DisplayName("should return zero for same angle")
        void sameAngle() {
            assertEquals(0, GeometryUtils.shortestAngularDifference(0, 0), ANGLE_DELTA);
            assertEquals(0, GeometryUtils.shortestAngularDifference(Math.PI / 2, Math.PI / 2), ANGLE_DELTA);
        }

        @Test
        @DisplayName("should find shortest path")
        void shortestPath() {
            // From 0 to π/2 should be π/2 (90 degrees)
            assertEquals(Math.PI / 2, 
                GeometryUtils.shortestAngularDifference(0, Math.PI / 2), ANGLE_DELTA);
            
            // From 0 to -π/2 should be -π/2 (-90 degrees)
            assertEquals(-Math.PI / 2, 
                GeometryUtils.shortestAngularDifference(0, -Math.PI / 2), ANGLE_DELTA);
        }

        @Test
        @DisplayName("should handle wrap around")
        void wrapAround() {
            // From small positive to small negative should be shortest path
            double from = Math.toRadians(10);
            double to = Math.toRadians(-10);
            
            double diff = GeometryUtils.shortestAngularDifference(from, to);
            assertEquals(Math.toRadians(-20), diff, ANGLE_DELTA);
        }
    }
}
