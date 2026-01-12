package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import org.dyn4j.geometry.Circle;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class RebuildFuelOnFly extends GamePieceProjectile {
  // see https://andymark.com/products/official-rebuilt-fuel
  public static final GamePieceInfo REBUILD_FUEL_INFO =
    new GamePieceInfo("Algae",
    new Circle(Inches.of(3).in(Meters)),
    Inches.of(6),
    Pounds.of(0.5),
    // These are from the 2025 Algae simulation values, they need to be updated.
    1.8, // Linear damping
    5, // Angular damping
    0.8); // coefficient of restitution

  private static Runnable hitNetCallBack = () -> System.out.println("hit target!");

  /**
  *
  *
  * <h2>Specifies a callback for when any ALGAE launched into the air hits the NET.</h2>
  *
  * @param callBack a {@link Runnable} to be invoked when an ALGAE hits the NET
  */
  public static void setHitNetCallBack(Runnable callBack) {
    hitNetCallBack = callBack;
  }

  public RebuildFuelOnFly(
    Translation2d robotPosition,
    Translation2d shooterPositionOnRobot,
    ChassisSpeeds chassisSpeeds,
    Rotation2d shooterFacing,
    Distance initialHeight,
    LinearVelocity launchingSpeed,
    Angle shooterAngle) {
      super(
        REBUILD_FUEL_INFO,
        robotPosition,
        shooterPositionOnRobot,
        chassisSpeeds,
        shooterFacing,
        initialHeight,
        launchingSpeed,
        shooterAngle);

      super.withTouchGroundHeight(REBUILD_FUEL_INFO.gamePieceHeight().in(Meters) * 0.5);
      super.enableBecomesGamePieceOnFieldAfterTouchGround();
  }
}

