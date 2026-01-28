package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Unit tests for Constants class.
 * Tests aim points, motor IDs, and constant validation.
 *
 * Note: Some tests related to DriverStation alliance are not included
 * as they require HAL initialization and alliance state mocking.
 */
@DisplayName("Constants")
class ConstantsTest {

    private static final double DELTA = TestUtils.DELTA;

    @Nested
    @DisplayName("AimPoints Enum")
    class AimPointsTests {

        @Test
        @DisplayName("should have distinct positions for red and blue alliances")
        void redAndBlueDistinct() {
            Translation3d redHub = Constants.AimPoints.RED_HUB.value;
            Translation3d blueHub = Constants.AimPoints.BLUE_HUB.value;

            assertNotEquals(redHub.getX(), blueHub.getX(),
                    "Red and blue hub X coordinates should differ");
        }

        @Test
        @DisplayName("red hub should be on right side of field (higher X)")
        void redHubPosition() {
            Translation3d redHub = Constants.AimPoints.RED_HUB.value;

            // Field is ~16.5m, red side is higher X values
            assertTrue(redHub.getX() > 8.0,
                    "Red hub X should be on right side of field (>8m)");
            assertTrue(redHub.getZ() > 0,
                    "Hub should be elevated (Z > 0)");
        }

        @Test
        @DisplayName("blue hub should be on left side of field (lower X)")
        void blueHubPosition() {
            Translation3d blueHub = Constants.AimPoints.BLUE_HUB.value;

            // Blue side is lower X values
            assertTrue(blueHub.getX() < 8.0,
                    "Blue hub X should be on left side of field (<8m)");
            assertTrue(blueHub.getZ() > 0,
                    "Hub should be elevated (Z > 0)");
        }

        @Test
        @DisplayName("hubs should be at same height (symmetric)")
        void hubsAtSameHeight() {
            Translation3d redHub = Constants.AimPoints.RED_HUB.value;
            Translation3d blueHub = Constants.AimPoints.BLUE_HUB.value;

            assertEquals(redHub.getZ(), blueHub.getZ(), DELTA,
                    "Red and blue hubs should be at same height");
        }

        @Test
        @DisplayName("hubs should be symmetric about field center Y")
        void hubsSymmetricInY() {
            Translation3d redHub = Constants.AimPoints.RED_HUB.value;
            Translation3d blueHub = Constants.AimPoints.BLUE_HUB.value;

            assertEquals(redHub.getY(), blueHub.getY(), DELTA,
                    "Red and blue hubs should have same Y coordinate (center of field)");
        }

        @ParameterizedTest
        @DisplayName("all aim points should have valid values")
        @EnumSource(Constants.AimPoints.class)
        void allAimPointsValid(Constants.AimPoints aimPoint) {
            Translation3d position = aimPoint.value;

            assertNotNull(position, "Aim point should not be null");

            // All positions should be on the field (roughly 0-17m X, 0-8m Y)
            assertTrue(position.getX() >= 0 && position.getX() <= 17,
                    String.format("%s X (%.2f) should be within field bounds", aimPoint, position.getX()));
            assertTrue(position.getY() >= 0 && position.getY() <= 9,
                    String.format("%s Y (%.2f) should be within field bounds", aimPoint, position.getY()));
            assertTrue(position.getZ() >= 0,
                    String.format("%s Z (%.2f) should not be negative", aimPoint, position.getZ()));
        }

        @Test
        @DisplayName("outpost positions should be at field edges")
        void outpostPositions() {
            Translation3d redOutpost = Constants.AimPoints.RED_OUTPOST.value;
            Translation3d blueOutpost = Constants.AimPoints.BLUE_OUTPOST.value;

            // Outposts are at corners of the field
            assertTrue(redOutpost.getX() > 14, "Red outpost should be near red wall");
            assertTrue(blueOutpost.getX() < 2, "Blue outpost should be near blue wall");
        }

        @Test
        @DisplayName("far side positions should be at opposite corners")
        void farSidePositions() {
            Translation3d redFarSide = Constants.AimPoints.RED_FAR_SIDE.value;
            Translation3d blueFarSide = Constants.AimPoints.BLUE_FAR_SIDE.value;

            // Far sides are at opposite Y values
            assertTrue(redFarSide.getX() > 14, "Red far side should be near red wall");
            assertTrue(blueFarSide.getX() < 2, "Blue far side should be near blue wall");

            // Far sides should have different Y than outposts
            assertNotEquals(redFarSide.getY(), Constants.AimPoints.RED_OUTPOST.value.getY(), DELTA,
                    "Red far side and outpost should have different Y");
        }
    }

    @Nested
    @DisplayName("Robot Physical Constants")
    class RobotConstantsTests {

        @Test
        @DisplayName("robot mass should be positive and reasonable")
        void robotMassReasonable() {
            assertTrue(Constants.ROBOT_MASS > 0, "Robot mass should be positive");
            assertTrue(Constants.ROBOT_MASS < 100, "Robot mass should be less than 100kg");
            assertTrue(Constants.ROBOT_MASS > 20, "Robot mass should be more than 20kg");
        }

        @Test
        @DisplayName("loop time should be positive and small")
        void loopTimeReasonable() {
            assertTrue(Constants.LOOP_TIME > 0, "Loop time should be positive");
            assertTrue(Constants.LOOP_TIME < 1, "Loop time should be less than 1 second");
        }

        @Test
        @DisplayName("max speed should be reasonable for FRC robot")
        void maxSpeedReasonable() {
            assertTrue(Constants.MAX_SPEED > 0, "Max speed should be positive");
            assertTrue(Constants.MAX_SPEED < 10, "Max speed should be less than 10 m/s");
            assertTrue(Constants.MAX_SPEED > 1, "Max speed should be more than 1 m/s");
        }
    }

    @Nested
    @DisplayName("Controller Constants")
    class ControllerConstantsTests {

        @Test
        @DisplayName("controller ports should be valid")
        void controllerPortsValid() {
            assertTrue(Constants.ControllerConstants.kDriverControllerPort >= 0,
                    "Driver port should be non-negative");
            assertTrue(Constants.ControllerConstants.kOperatorControllerPort >= 0,
                    "Operator port should be non-negative");

            assertNotEquals(
                    Constants.ControllerConstants.kDriverControllerPort,
                    Constants.ControllerConstants.kOperatorControllerPort,
                    "Controller ports should be different");
        }

        @Test
        @DisplayName("deadband should be reasonable")
        void deadbandReasonable() {
            assertTrue(Constants.ControllerConstants.DEADBAND > 0,
                    "Deadband should be positive");
            assertTrue(Constants.ControllerConstants.DEADBAND < 0.5,
                    "Deadband should be less than 0.5");
            assertTrue(Constants.ControllerConstants.DEADBAND >= 0.05,
                    "Deadband should be at least 0.05");
        }
    }

    @Nested
    @DisplayName("Motor ID Constants")
    class MotorIdTests {

        @Test
        @DisplayName("all swerve motor IDs should be unique")
        void swerveIdsUnique() {
            int[] ids = {
                    Constants.DriveConstants.FL.kTurnMotorId,
                    Constants.DriveConstants.FL.kDriveMotorId,
                    Constants.DriveConstants.FR.kTurnMotorId,
                    Constants.DriveConstants.FR.kDriveMotorId,
                    Constants.DriveConstants.BL.kTurnMotorId,
                    Constants.DriveConstants.BL.kDriveMotorId,
                    Constants.DriveConstants.BR.kTurnMotorId,
                    Constants.DriveConstants.BR.kDriveMotorId
            };

            // Check all IDs are unique
            for (int i = 0; i < ids.length; i++) {
                for (int j = i + 1; j < ids.length; j++) {
                    assertNotEquals(ids[i], ids[j],
                            String.format("Motor IDs at positions %d and %d should be different", i, j));
                }
            }
        }

        @Test
        @DisplayName("all absolute encoder IDs should be unique")
        void encoderIdsUnique() {
            int[] ids = {
                    Constants.DriveConstants.FL.kAbsId,
                    Constants.DriveConstants.FR.kAbsId,
                    Constants.DriveConstants.BL.kAbsId,
                    Constants.DriveConstants.BR.kAbsId
            };

            for (int i = 0; i < ids.length; i++) {
                for (int j = i + 1; j < ids.length; j++) {
                    assertNotEquals(ids[i], ids[j],
                            String.format("Encoder IDs at positions %d and %d should be different", i, j));
                }
            }
        }

        @Test
        @DisplayName("shooter motor IDs should be unique")
        void shooterIdsUnique() {
            assertNotEquals(
                    Constants.ShooterConstants.kLeaderMotorId,
                    Constants.ShooterConstants.kFollowerMotorId,
                    "Shooter leader and follower should have different IDs");
        }

        @Test
        @DisplayName("all subsystem motor IDs should be unique across subsystems")
        void allSubsystemIdsUnique() {
            int[] allIds = {
                    Constants.ShooterConstants.kLeaderMotorId,
                    Constants.ShooterConstants.kFollowerMotorId,
                    Constants.TurretConstants.kMotorId,
                    Constants.HoodConstants.kMotorId,
                    Constants.IntakeConstants.kPivotMotorId,
                    Constants.IntakeConstants.kRollerMotorId,
                    Constants.HopperConstants.kHopperMotorId,
                    Constants.KickerConstants.kKickerMotorId
            };

            for (int i = 0; i < allIds.length; i++) {
                for (int j = i + 1; j < allIds.length; j++) {
                    assertNotEquals(allIds[i], allIds[j],
                            String.format("Motor IDs %d and %d at positions %d and %d should be different",
                                    allIds[i], allIds[j], i, j));
                }
            }
        }

        @Test
        @DisplayName("all motor IDs should be positive")
        void allIdsPositive() {
            assertTrue(Constants.ShooterConstants.kLeaderMotorId > 0, "IDs should be positive");
            assertTrue(Constants.ShooterConstants.kFollowerMotorId > 0, "IDs should be positive");
            assertTrue(Constants.TurretConstants.kMotorId > 0, "IDs should be positive");
            assertTrue(Constants.HoodConstants.kMotorId > 0, "IDs should be positive");
            assertTrue(Constants.IntakeConstants.kPivotMotorId > 0, "IDs should be positive");
            assertTrue(Constants.IntakeConstants.kRollerMotorId > 0, "IDs should be positive");
            assertTrue(Constants.HopperConstants.kHopperMotorId > 0, "IDs should be positive");
            assertTrue(Constants.KickerConstants.kKickerMotorId > 0, "IDs should be positive");
        }
    }
}
