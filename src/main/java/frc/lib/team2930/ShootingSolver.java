package frc.lib.team2930;

import edu.wpi.first.math.geometry.*;

public class ShootingSolver {
  public static final int DebugSpew = 0;

  private static final LoggerGroup logGroup = LoggerGroup.build("ShootingSolver");
  private static final LoggerEntry.Decimal logThetaGamepiece =
      logGroup.buildDecimal("thetaGamepiece");
  private static final LoggerEntry.Decimal logTargetHeading =
      logGroup.buildDecimal("targetHeading");

  private final Translation3d Ptarget;
  private final Translation3d PaxisOfRotationShooter;
  private final Translation3d PfrontOfShooter;
  private final double shooterSpeed;
  private final double shootingTime;
  private final boolean correctForLateralMotion;
  private double startOfShootingTimestamp = Double.NaN;
  private boolean doneShooting;

  public record Solution(
      double timeToShoot,
      Rotation2d heading,
      double rotationSpeed,
      Rotation2d pitch,
      double xyDistance,
      Translation3d xyOffset) {}

  public ShootingSolver(
      Translation3d Ptarget,
      Translation3d PaxisOfRotationShooter,
      Translation3d PfrontOfShooter,
      double shooterSpeed,
      double shootingTime,
      boolean correctForLateralMotion) {
    this.Ptarget = Ptarget;
    this.PaxisOfRotationShooter = PaxisOfRotationShooter;
    this.PfrontOfShooter = PfrontOfShooter;
    this.shooterSpeed = shooterSpeed;
    this.shootingTime = shootingTime;
    this.correctForLateralMotion = correctForLateralMotion;
  }

  public void startShooting(double timestamp) {
    startOfShootingTimestamp = timestamp;
  }

  public void endShooting() {
    startOfShootingTimestamp = Double.NaN;
    doneShooting = false;
  }

  public boolean isShooting() {
    return Double.isFinite(startOfShootingTimestamp) && !doneShooting;
  }

  /**
   * @return robot theta and shooter pitch
   */
  public Solution computeAngles(double currentTime, Pose2d robotPose, Translation2d robotVel) {
    double timeToShoot;
    if (Double.isNaN(this.startOfShootingTimestamp)) {
      timeToShoot = shootingTime;
    } else {
      timeToShoot = shootingTime - (currentTime - startOfShootingTimestamp);
      if (timeToShoot < 0) {
        doneShooting = true;
        timeToShoot = 0;
      }
    }

    // 3d position of robot
    var Probot = GeometryUtil.translation2dTo3d(robotPose.getTranslation());

    // 3d velocity vector of robot
    var Vrobot = GeometryUtil.translation2dTo3d(robotVel);

    // Position of robot when gamepiece leaves shooter
    var ProbotFuture = Probot.plus(Vrobot.times(timeToShoot));
    if (DebugSpew > 0) {
      System.out.printf("ProbotFuture: %s\n", ProbotFuture);
    }

    // Vector pointing from robot to target at time that gamepiece leaves shooter
    var dPtarget = Ptarget.minus(ProbotFuture);

    // Direction of the gamepiece to face the target.
    var thetaGamepiece = Math.atan2(dPtarget.getY(), dPtarget.getX());
    logThetaGamepiece.info(Math.toDegrees(thetaGamepiece));

    // Robot relative translation of axis of rotation of shooter
    var Paxis = PaxisOfRotationShooter.rotateBy(new Rotation3d(0.0, 0.0, thetaGamepiece));

    // Position of axis when gamepiece leaves shooter
    var PaxisFuture = ProbotFuture.plus(Paxis);

    // Vector from the gamepiece to the target. We need to align with this vector to score.
    var dPtargetAxis = Ptarget.minus(PaxisFuture);

    double dPtargetAxisX = dPtargetAxis.getX();
    double dPtargetAxisY = dPtargetAxis.getY();

    // Direction to the target from the gamepiece.
    double targetHeading = Math.atan2(dPtargetAxisY, dPtargetAxisX);

    logTargetHeading.info(Math.toDegrees(targetHeading));

    // Horizontal dist to target
    var xyDistToTarget = Math.hypot(dPtargetAxisX, dPtargetAxisY);
    if (DebugSpew > 0) {
      System.out.println();
      System.out.printf("Target: heading:%.1f\n", Math.toDegrees(targetHeading));
      System.out.printf("         dx:%.1f\n", dPtargetAxisX);
      System.out.printf("         dy:%.1f\n", dPtargetAxisY);
      System.out.printf("         xyDistToTarget: %.1f\n", xyDistToTarget);

      System.out.printf("Robot: Vx:%.1f\n", robotVel.getX());
      System.out.printf("       Vy:%.1f\n", robotVel.getY());
    }

    // Desired pitch of the gamepiece trajectory
    double pitchGamepiece = Math.atan2(dPtargetAxis.getZ(), xyDistToTarget);

    // We start assuming the pitch of the shooter is going to be the pitch of the trajectory
    double shooterPitch = pitchGamepiece;

    // Maximum number of approximations to perform
    int maxApproximations = 10;

    double targetTheta = targetHeading;

    if (correctForLateralMotion) {
      // We need to compute this, which depends on shooterPitch, so we need to use successive
      // approximations
      double yawInNewFrame;

      while (true) {
        //
        // The system of equations to solve for cancelling the horizontal component of the robot
        // velocity is:
        //
        // Vn_x = (Vr_x + Vs * cos(targetTheta))
        // Vn_y = (Vr_y + Vs * sin(targetTheta))
        //
        // The Vn vector has to be aligned with dPtargetAxis vector, so
        //
        // Vn_x = c * dPtargetAxisX
        // Vn_y = c * dPtargetAxisY
        //
        // To make it easier, we rotate the reference frame such that the Y velocity will be zero:
        //
        //     Vn_y = 0
        // ->  Vr_y + Vs * sin(targetTheta) = 0
        // -> -Vr_y / Vs = sin(targetTheta))
        // ->  targetTheta = arcsin(-Vr_y / Vs)
        //
        // To verify that the condition is physically possible, we check that the gamepiece moves
        // towards
        // the
        // target:
        //
        // Vn_x = (Vr_x + Vs * cos(targetTheta)) > 0
        //
        if (DebugSpew > 1) {
          System.out.printf(
              "shooterPitch:%.1f  maxApproximations:%d\n",
              Math.toDegrees(shooterPitch), maxApproximations);
        }
        var VgamepieceVertical = shooterSpeed * Math.sin(shooterPitch);
        var VgamepieceHorizontal = shooterSpeed * Math.cos(shooterPitch);
        var gamepieceRelativeVel = new Translation2d(VgamepieceHorizontal, 0);

        Rotation2d frameRotation = Rotation2d.fromRadians(-targetHeading);
        var robotVelInNewFrame = robotVel.rotateBy(frameRotation);
        var gamepieceRelativeVelInNewFrame = gamepieceRelativeVel.rotateBy(frameRotation);

        double robotVelXInNewFrame = robotVelInNewFrame.getX();
        double robotVelYInNewFrame = robotVelInNewFrame.getY();
        double gamepieceVelXInNewFrame = gamepieceRelativeVelInNewFrame.getX();
        double gamepieceVelYInNewFrame = gamepieceRelativeVelInNewFrame.getY();

        if (DebugSpew > 1) {
          System.out.printf("NewFrame: robotVelX:%.1f\n", robotVelXInNewFrame);
          System.out.printf("          robotVelY:%.1f\n", robotVelYInNewFrame);
          System.out.printf("          gamepieceVelX :%.1f\n", gamepieceVelXInNewFrame);
          System.out.printf("          gamepieceVelY :%.1f\n", gamepieceVelYInNewFrame);
        }

        yawInNewFrame = Math.asin(-robotVelYInNewFrame / VgamepieceHorizontal);

        if (Double.isNaN(yawInNewFrame)
            || (robotVelXInNewFrame + VgamepieceHorizontal * Math.cos(yawInNewFrame) <= 0)) {
          return null;
        }

        var VgamepieceHorizontal_FieldRelative =
            robotVelXInNewFrame + VgamepieceHorizontal * Math.cos(yawInNewFrame);

        if (DebugSpew > 2) // Check equation outputs.
        {
          double robotVelX = robotVel.getX();
          double robotVelY = robotVel.getY();
          double targetTheta2 = targetHeading + yawInNewFrame;

          double gamepieceX = robotVelX + VgamepieceHorizontal * Math.cos(targetTheta2);
          double gamepieceY = robotVelY + VgamepieceHorizontal * Math.sin(targetTheta2);
          double nodeHeading = Math.atan2(gamepieceY, gamepieceX);
          System.out.printf(
              "gamepieceHeading: %.1f  gamepieceX:%.1f gamepieceY:%.1f\n",
              Math.toDegrees(nodeHeading), gamepieceX, gamepieceY);
        }

        // We have to verify that after the adjustment the gamepiece is still pointing to the
        // target

        // Desired pitch of the gamepiece trajectory
        double pitchGamepieceNew =
            Math.atan2(VgamepieceVertical, VgamepieceHorizontal_FieldRelative);

        if (DebugSpew > 1) {
          System.out.printf("Gamepiece: NewPitch:%.1f\n", Math.toDegrees(pitchGamepieceNew));
          System.out.printf("      DesiredPitch:%.1f\n", Math.toDegrees(pitchGamepiece));
          System.out.printf("      Yaw:%.1f\n", Math.toDegrees(yawInNewFrame));
          System.out.printf("      VgamepieceVertical:%.1f\n", VgamepieceVertical);
          System.out.printf("      VgamepieceHorizontal:%.1f\n", VgamepieceHorizontal);
          System.out.printf(
              "      VgamepieceHorizontalFieldRelative:%.1f\n", VgamepieceHorizontal_FieldRelative);
        }

        if (Math.toDegrees(Math.abs(pitchGamepieceNew - pitchGamepiece)) < 0.1) {
          break;
        }

        shooterPitch -= (pitchGamepieceNew - pitchGamepiece);

        if (maxApproximations-- <= 0) {
          break;
        }
      }

      targetTheta += yawInNewFrame;
    }

    var PaxisFinal = PaxisOfRotationShooter.rotateBy(new Rotation3d(0.0, 0.0, targetTheta));
    var PfrontFinal = PfrontOfShooter.rotateBy(new Rotation3d(0.0, 0.0, targetTheta));

    double axisDistance = Ptarget.getDistance(ProbotFuture.plus(PaxisFinal));
    double frontDistance = Ptarget.getDistance(ProbotFuture.plus(PfrontFinal));

    if (frontDistance > axisDistance) {
      targetTheta += Math.PI;
    }

    var currentHeading = robotPose.getRotation().getRadians();

    var deltaAngle = GeometryUtil.optimizeRotation(targetTheta - currentHeading);

    var rateOfRotation = deltaAngle / Math.max(0.01, timeToShoot);

    return new Solution(
        timeToShoot,
        new Rotation2d(targetTheta),
        rateOfRotation,
        new Rotation2d(shooterPitch),
        xyDistToTarget,
        dPtarget);
  }
}
