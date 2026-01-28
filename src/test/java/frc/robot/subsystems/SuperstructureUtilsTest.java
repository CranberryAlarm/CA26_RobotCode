package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.TestUtils;
import frc.robot.subsystems.SuperstructureUtils.ReadinessStatus;

/**
 * Unit tests for SuperstructureUtils.
 * Tests tolerance checking, readiness calculations, and error calculations.
 *
 * Following best practices:
 * - Each test method tests one specific behavior
 * - Tests are organized by feature using nested classes
 * - Edge cases and boundary conditions are tested
 * - Uses parameterized tests for data-driven testing
 */
@DisplayName("SuperstructureUtils")
class SuperstructureUtilsTest {

    private static final double DELTA = TestUtils.DELTA;

    @Nested
    @DisplayName("Shooter Speed Tolerance")
    class ShooterToleranceTests {

        @Test
        @DisplayName("should return true when at exact target speed")
        void shooterAtExactTarget() {
            AngularVelocity current = RPM.of(3000);
            AngularVelocity target = RPM.of(3000);

            assertTrue(SuperstructureUtils.isShooterAtSpeed(current, target),
                    "Should be at speed when exactly at target");
        }

        @Test
        @DisplayName("should return true when just under tolerance")
        void shooterJustUnderTolerance() {
            AngularVelocity target = RPM.of(3000);
            AngularVelocity current = RPM.of(3000 + 99); // Just under 100 RPM tolerance

            assertTrue(SuperstructureUtils.isShooterAtSpeed(current, target),
                    "Should be at speed when 99 RPM away (under 100 RPM tolerance)");
        }

        @Test
        @DisplayName("should return false when outside tolerance")
        void shooterAtToleranceBoundary() {
            AngularVelocity target = RPM.of(3000);
            AngularVelocity current = RPM.of(3000 + 101); // Just outside tolerance

            assertFalse(SuperstructureUtils.isShooterAtSpeed(current, target),
                    "Should not be at speed when 101 RPM away (outside 100 RPM tolerance)");
        }

        @Test
        @DisplayName("should return false when outside tolerance")
        void shooterOutsideTolerance() {
            AngularVelocity target = RPM.of(3000);
            AngularVelocity current = RPM.of(3000 + 200); // Well outside tolerance

            assertFalse(SuperstructureUtils.isShooterAtSpeed(current, target),
                    "Should not be at speed when 200 RPM away");
        }

        @ParameterizedTest
        @DisplayName("should handle various speed differences correctly")
        @CsvSource({
                "3000, 3000, true", // Exact
                "3000, 3050, true", // Within tolerance (50 RPM)
                "3000, 3099, true", // Just inside tolerance (99 < 101)
                "3000, 3101, false", // Outside boundary (101 > 100)
                "3000, 3200, false", // Outside
                "3000, 2901, true", // Below target, within tolerance (99 < 100)
                "3000, 2899, false", // Below target, outside boundary (101 > 100)
                "3000, 2800, false" // Below target, outside
        })
        void shooterToleranceParameterized(double target, double current, boolean expected) {
            AngularVelocity currentSpeed = RPM.of(current);
            AngularVelocity targetSpeed = RPM.of(target);

            assertEquals(expected, SuperstructureUtils.isShooterAtSpeed(currentSpeed, targetSpeed),
                    String.format("Current: %.0f, Target: %.0f should be %s", current, target, expected));
        }

        @Test
        @DisplayName("should accept custom tolerance")
        void shooterCustomTolerance() {
            AngularVelocity current = RPM.of(3000);
            AngularVelocity target = RPM.of(3150);
            AngularVelocity tolerance = RPM.of(200); // Larger tolerance

            assertTrue(SuperstructureUtils.isShooterAtSpeed(current, target, tolerance),
                    "Should be at speed with custom larger tolerance");
        }
    }

    @Nested
    @DisplayName("Turret Angle Tolerance")
    class TurretToleranceTests {

        @Test
        @DisplayName("should return true when at exact target angle")
        void turretAtExactTarget() {
            Angle current = Degrees.of(45);
            Angle target = Degrees.of(45);

            assertTrue(SuperstructureUtils.isTurretOnTarget(current, target),
                    "Should be on target when exactly at target angle");
        }

        @Test
        @DisplayName("should return true when within 1 degree tolerance")
        void turretWithinTolerance() {
            Angle target = Degrees.of(45);
            Angle current = Degrees.of(45.5); // 0.5 degrees away

            assertTrue(SuperstructureUtils.isTurretOnTarget(current, target),
                    "Should be on target when 0.5 degrees away");
        }

        @Test
        @DisplayName("should return false when at 1 degree boundary")
        void turretAtBoundary() {
            Angle target = Degrees.of(45);
            Angle current = Degrees.of(46); // Exactly 1 degree away

            assertFalse(SuperstructureUtils.isTurretOnTarget(current, target),
                    "Should not be on target when exactly at 1 degree boundary");
        }

        @ParameterizedTest
        @DisplayName("should handle positive and negative angles")
        @CsvSource({
                "0, 0, true",
                "45, 45.5, true",
                "-45, -45.5, true",
                "90, 89.5, true",
                "-90, -89, false",
                "0, 1.5, false"
        })
        void turretToleranceParameterized(double current, double target, boolean expected) {
            Angle currentAngle = Degrees.of(current);
            Angle targetAngle = Degrees.of(target);

            assertEquals(expected, SuperstructureUtils.isTurretOnTarget(currentAngle, targetAngle),
                    String.format("Current: %.1f°, Target: %.1f° should be %s", current, target, expected));
        }

        @Test
        @DisplayName("should accept custom tolerance")
        void turretCustomTolerance() {
            Angle current = Degrees.of(45);
            Angle target = Degrees.of(48);
            Angle tolerance = Degrees.of(5);

            assertTrue(SuperstructureUtils.isTurretOnTarget(current, target, tolerance),
                    "Should be on target with custom larger tolerance");
        }
    }

    @Nested
    @DisplayName("Hood Angle Tolerance")
    class HoodToleranceTests {

        @Test
        @DisplayName("should return true when within 2 degree tolerance")
        void hoodWithinTolerance() {
            Angle target = Degrees.of(30);
            Angle current = Degrees.of(31.5); // 1.5 degrees away

            assertTrue(SuperstructureUtils.isHoodOnTarget(current, target),
                    "Should be on target when 1.5 degrees away (within 2 degree tolerance)");
        }

        @Test
        @DisplayName("should return false when at 2 degree boundary")
        void hoodAtBoundary() {
            Angle target = Degrees.of(30);
            Angle current = Degrees.of(32); // Exactly 2 degrees away

            assertFalse(SuperstructureUtils.isHoodOnTarget(current, target),
                    "Should not be on target when exactly at 2 degree boundary");
        }

        @ParameterizedTest
        @DisplayName("should handle typical hood angles")
        @CsvSource({
                "15, 15, true", // Exact
                "30, 31, true", // 1 deg diff < 2 deg tolerance
                "45, 43.5, true", // 1.5 deg diff < 2 deg tolerance (changed from 43 which was exactly 2)
                "60, 62.5, false", // 2.5 deg diff >= 2 deg tolerance
                "75, 75, true", // Exact
                "90, 87, false" // 3 deg diff >= 2 deg tolerance
        })
        void hoodToleranceParameterized(double current, double target, boolean expected) {
            Angle currentAngle = Degrees.of(current);
            Angle targetAngle = Degrees.of(target);

            assertEquals(expected, SuperstructureUtils.isHoodOnTarget(currentAngle, targetAngle),
                    String.format("Current: %.1f°, Target: %.1f° should be %s", current, target, expected));
        }
    }

    @Nested
    @DisplayName("Combined Readiness Check")
    class ReadinessTests {

        @Test
        @DisplayName("should return true when all mechanisms ready")
        void allReady() {
            assertTrue(SuperstructureUtils.isReadyToShoot(
                    RPM.of(3000), RPM.of(3000), // Shooter
                    Degrees.of(45), Degrees.of(45), // Turret
                    Degrees.of(30), Degrees.of(30) // Hood
            ), "Should be ready when all mechanisms at target");
        }

        @Test
        @DisplayName("should return false when shooter not ready")
        void shooterNotReady() {
            assertFalse(SuperstructureUtils.isReadyToShoot(
                    RPM.of(2500), RPM.of(3000), // Shooter NOT ready (500 RPM off)
                    Degrees.of(45), Degrees.of(45), // Turret ready
                    Degrees.of(30), Degrees.of(30) // Hood ready
            ), "Should not be ready when shooter off target");
        }

        @Test
        @DisplayName("should return false when turret not ready")
        void turretNotReady() {
            assertFalse(SuperstructureUtils.isReadyToShoot(
                    RPM.of(3000), RPM.of(3000), // Shooter ready
                    Degrees.of(40), Degrees.of(45), // Turret NOT ready (5 degrees off)
                    Degrees.of(30), Degrees.of(30) // Hood ready
            ), "Should not be ready when turret off target");
        }

        @Test
        @DisplayName("should return false when hood not ready")
        void hoodNotReady() {
            assertFalse(SuperstructureUtils.isReadyToShoot(
                    RPM.of(3000), RPM.of(3000), // Shooter ready
                    Degrees.of(45), Degrees.of(45), // Turret ready
                    Degrees.of(25), Degrees.of(30) // Hood NOT ready (5 degrees off)
            ), "Should not be ready when hood off target");
        }

        @Test
        @DisplayName("should return false when multiple mechanisms not ready")
        void multipleNotReady() {
            assertFalse(SuperstructureUtils.isReadyToShoot(
                    RPM.of(2500), RPM.of(3000), // Shooter NOT ready
                    Degrees.of(40), Degrees.of(45), // Turret NOT ready
                    Degrees.of(25), Degrees.of(30) // Hood NOT ready
            ), "Should not be ready when multiple mechanisms off target");
        }

        @Test
        @DisplayName("should allow small errors within all tolerances")
        void allWithinTolerance() {
            assertTrue(SuperstructureUtils.isReadyToShoot(
                    RPM.of(3050), RPM.of(3000), // Shooter 50 RPM off (within 100)
                    Degrees.of(45.5), Degrees.of(45), // Turret 0.5° off (within 1°)
                    Degrees.of(31), Degrees.of(30) // Hood 1° off (within 2°)
            ), "Should be ready when all within tolerance");
        }
    }

    @Nested
    @DisplayName("Error Calculations")
    class ErrorCalculationTests {

        @Test
        @DisplayName("should return zero error when at target")
        void zeroErrorAtTarget() {
            assertEquals(0, SuperstructureUtils.getShooterError(RPM.of(3000), RPM.of(3000)), DELTA);
            assertEquals(0, SuperstructureUtils.getTurretError(Degrees.of(45), Degrees.of(45)), DELTA);
            assertEquals(0, SuperstructureUtils.getHoodError(Degrees.of(30), Degrees.of(30)), DELTA);
        }

        @Test
        @DisplayName("should return positive error when below target")
        void positiveErrorWhenBelowTarget() {
            // Shooter running slow
            double shooterError = SuperstructureUtils.getShooterError(RPM.of(2800), RPM.of(3000));
            assertEquals(200, shooterError, DELTA, "Error should be +200 (need to speed up)");

            // Turret needs to turn more
            double turretError = SuperstructureUtils.getTurretError(Degrees.of(40), Degrees.of(45));
            assertEquals(5, turretError, DELTA, "Error should be +5°");
        }

        @Test
        @DisplayName("should return negative error when above target")
        void negativeErrorWhenAboveTarget() {
            // Shooter running fast
            double shooterError = SuperstructureUtils.getShooterError(RPM.of(3200), RPM.of(3000));
            assertEquals(-200, shooterError, DELTA, "Error should be -200 (need to slow down)");

            // Turret past target
            double turretError = SuperstructureUtils.getTurretError(Degrees.of(50), Degrees.of(45));
            assertEquals(-5, turretError, DELTA, "Error should be -5°");
        }
    }

    @Nested
    @DisplayName("ReadinessStatus Record")
    class ReadinessStatusTests {

        @Test
        @DisplayName("should provide detailed status when all ready")
        void detailedStatusAllReady() {
            ReadinessStatus status = SuperstructureUtils.getReadinessStatus(
                    RPM.of(3000), RPM.of(3000),
                    Degrees.of(45), Degrees.of(45),
                    Degrees.of(30), Degrees.of(30));

            assertTrue(status.shooterReady(), "Shooter should be ready");
            assertTrue(status.turretReady(), "Turret should be ready");
            assertTrue(status.hoodReady(), "Hood should be ready");
            assertTrue(status.isReady(), "Overall should be ready");

            assertEquals(0, status.shooterError(), DELTA, "Shooter error should be 0");
            assertEquals(0, status.turretError(), DELTA, "Turret error should be 0");
            assertEquals(0, status.hoodError(), DELTA, "Hood error should be 0");
        }

        @Test
        @DisplayName("should provide detailed status when partially ready")
        void detailedStatusPartiallyReady() {
            ReadinessStatus status = SuperstructureUtils.getReadinessStatus(
                    RPM.of(3000), RPM.of(3000), // Ready
                    Degrees.of(40), Degrees.of(45), // Not ready (5° off)
                    Degrees.of(30), Degrees.of(30) // Ready
            );

            assertTrue(status.shooterReady(), "Shooter should be ready");
            assertFalse(status.turretReady(), "Turret should NOT be ready");
            assertTrue(status.hoodReady(), "Hood should be ready");
            assertFalse(status.isReady(), "Overall should NOT be ready");

            assertEquals(5, status.turretError(), DELTA, "Turret error should be 5°");
        }

        @Test
        @DisplayName("should track errors even when ready")
        void errorsTrackedWhenReady() {
            ReadinessStatus status = SuperstructureUtils.getReadinessStatus(
                    RPM.of(3050), RPM.of(3000), // Ready but 50 RPM off
                    Degrees.of(45.5), Degrees.of(45), // Ready but 0.5° off
                    Degrees.of(31), Degrees.of(30) // Ready but 1° off
            );

            assertTrue(status.isReady(), "Should be ready overall");
            assertEquals(-50, status.shooterError(), DELTA, "Should track 50 RPM error");
            assertEquals(-0.5, status.turretError(), DELTA, "Should track 0.5° error");
            assertEquals(-1, status.hoodError(), DELTA, "Should track 1° error");
        }
    }

    @Nested
    @DisplayName("Constant Values")
    class ConstantTests {

        @Test
        @DisplayName("should have correct default tolerances")
        void defaultTolerances() {
            assertEquals(100, SuperstructureUtils.SHOOTER_TOLERANCE.in(RPM), DELTA,
                    "Shooter tolerance should be 100 RPM");
            assertEquals(1, SuperstructureUtils.TURRET_TOLERANCE.in(Degrees), DELTA,
                    "Turret tolerance should be 1 degree");
            assertEquals(2, SuperstructureUtils.HOOD_TOLERANCE.in(Degrees), DELTA,
                    "Hood tolerance should be 2 degrees");
        }
    }
}
