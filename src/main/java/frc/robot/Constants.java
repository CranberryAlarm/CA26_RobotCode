package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import swervelib.math.Matter;

public final class Constants {

  public static enum AimPoints {
    RED_HUB(new Translation3d(11.938, 4.034536, 1.5748)),
    RED_OUTPOST(new Translation3d(15.75, 7.25, 0)),
    RED_FAR_SIDE(new Translation3d(15.75, 0.75, 0)),

    BLUE_HUB(new Translation3d(4.5974, 4.034536, 1.5748)),
    BLUE_OUTPOST(new Translation3d(0.75, 0.75, 0)),
    BLUE_FAR_SIDE(new Translation3d(0.75, 7.25, 0));

    public final Translation3d value;

    private AimPoints(Translation3d value) {
      this.value = value;
    }

    public static final Translation3d getAllianceHubPosition() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_HUB.value : BLUE_HUB.value;
    }

    public static final Translation3d getAllianceOutpostPosition() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_OUTPOST.value : BLUE_OUTPOST.value;
    }

    public static final Translation3d getAllianceFarSidePosition() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_FAR_SIDE.value : BLUE_FAR_SIDE.value;
    }
  }

  public static final double ROBOT_MASS = Units.lbsToKilograms(120); // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms spark max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kPoseControllerPort = 2;

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
  }

  public static class DriveConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static class FL {
      public static final int kTurnMotorId = 5;
      public static final int kDriveMotorId = 6;
      public static final int kAbsId = 0;
    }

    public static class FR {
      public static final int kTurnMotorId = 9;
      public static final int kDriveMotorId = 10;
      public static final int kAbsId = 2;
    }

    public static class BL {
      public static final int kTurnMotorId = 7;
      public static final int kDriveMotorId = 8;
      public static final int kAbsId = 1;
    }

    public static class BR {
      public static final int kTurnMotorId = 11;
      public static final int kDriveMotorId = 12;
      public static final int kAbsId = 3;
    }
  }

  public static class ShooterConstants {
    // 2 Neos, 4in shooter wheels
    public static final int kLeaderMotorId = 15;
    public static final int kFollowerMotorId = 16;
  }

  public static class TurretConstants {
    // 1 Neo, 6.875 in diameter, 4:1 gearbox, 10:1 pivot gearing, non-continuous
    // 360 deg
    public static final int kMotorId = 17;
  }

  public static class HoodConstants {
    // 1 Neo, 0-90 degree variability, 50:1 reduction
    public static final int kMotorId = 19;
  }

  // Intake subsystem CAN IDs start at 30
  public static class IntakeConstants {
    // SparkFlex controlling the intake flywheel
    public static final int kPivotMotorId = 30;
    public static final int kRollerMotorId = 31;
  }

  // Hopper subsystem CAN IDs start at 40
  public static class HopperConstants {
    public static final int kHopperMotorId = 40;
  }

  // Kicker subsystem CAN IDs start at 50
  public static class KickerConstants {
    public static final int kKickerMotorId = 50;
  }
}
