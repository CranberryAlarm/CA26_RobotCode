package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {

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

  public static class AlgaeConstants {
    public static final int kWristMotorId = 13;
    public static final int kIntakeMotorId = 14;
  }

  public static class CoralConstants {
    public static final int kLeftIndexMotorId = 11;
    public static final int kRightIndexMotorId = 12;

    public static final int kIndexLaserCANId = 0;
  }

  public static class ShooterConstants {
    // 2 Neos, 4in shooter wheels, 4:1 gearbox reduction
    public static final int kLeaderMotorId = 15;
    public static final int kFollowerMotorId = 16;
  }

  public static class TurretConstants {
    // 2 Neos, 12in diameter, 25:1 gearbox, 10:1 pivot gearing, non-continuous (270
    // FOV)
    public static final int kLeaderMotorId = 17;
    public static final int kFollowerMotorId = 18;
  }

  public static class HoodConstants {
    // 1 Neo, 0-90 degree variability, 50:1 reduction
    public static final int kMotorId = 19;
  }
}
