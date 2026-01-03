package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
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
}
