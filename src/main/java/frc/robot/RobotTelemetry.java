package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;

public class RobotTelemetry {

  @SuppressWarnings("resource")
  public RobotTelemetry() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // TODO: this
    // Logger.recordMetadata("RobotType", RobotConstants.getRobotType().name());
    // Logger.recordMetadata("ProjectName", "Apollo, Eater of Batteries");

    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

    Logger.start();

    print("Logging initialized. Fard.");
  }

  public static void print(String output) {
    System.out.println(
        String.format("%.3f", Timer.getFPGATimestamp()) + " || " + output);
  }

  public static void print(Object output) {
    System.out.println(
        String.format("%.3f", Timer.getFPGATimestamp()) + " || " + output.toString());
  }
}
