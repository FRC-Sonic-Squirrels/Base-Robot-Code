// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ArrayUtil;
import frc.lib.team2930.AutoLock;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team6328.PoseEstimator;
import frc.lib.team6328.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.Robot;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {
  public static final AutoLock odometryLock = new AutoLock();

  private final GyroIO gyroIO;
  // private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final SwerveModules modules;

  private final SwerveDriveKinematics kinematics;
  private Pose2d rawOdometryPose = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();

  private final PoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();
  private final Field2d rawOdometryField2d = new Field2d();

  RobotConfig config;

  public Drivetrain(RobotConfig config, GyroIO gyroIO, SwerveModules swerveModules) {
    this.config = config;
    this.modules = swerveModules;
    this.gyroIO = gyroIO;

    kinematics = config.getSwerveDriveKinematics();

    // FIXME: values copied from 6328, learn how to calculate these values
    poseEstimator = new PoseEstimator(0.1, 0.1, 0.3);

    // Configure AutoBuilder for PathPlanner
    // FIXME: pass in custom PID constants? Issue for this use case has been created:
    // https://github.com/mjansen4857/pathplanner/issues/474
    // FIXME: fix all the pathplanner jank
    // AutoBuilder.configureHolonomic(
    //   this::getPose,
    //   this::setPose,
    //   () -> kinematics.toChassisSpeeds(getModuleStates()),
    //   this::runVelocity,
    //   new HolonomicPathFollowerConfig(
    //     getMaxAngularSpeedRadPerSec(),
    //     getCharacterizationVelocity(),
    //     new ReplanningConfig()),
    //   () -> true,
    //   this);

    // // FIXME:
    // Pathfinding.setPathfinder(new LocalADStarAK());
    // PathPlannerLogging.setLogActivePathCallback(
    //     (activePath) -> {
    //       Logger.recordOutput(
    //           "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
    //     });
    // PathPlannerLogging.setLogTargetPoseCallback(
    //     (targetPose) -> {
    //       Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
    //     });

  }

  private Translation2d simulatedAcceleration = new Translation2d();
  private Translation2d lastVel = new Translation2d();

  public void periodic() {
    try (var ignored = new ExecutionTiming("Drivetrain")) {
      try (var ignored2 = odometryLock.lock()) // Prevents odometry updates while reading data
      {
        gyroIO.updateInputs(gyroInputs);
        modules.updateInputs();
      }

      Logger.processInputs("Drive/Gyro", gyroInputs);
      modules.periodic();

      // Stop moving when disabled
      if (DriverStation.isDisabled()) {
        modules.stop();
      }
      // Log empty setpoint states when disabled
      if (DriverStation.isDisabled()) {
        Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
        Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
      }

      // Update odometry
      int deltaCount =
          gyroInputs.connected ? gyroInputs.odometryYawPositions.length : Integer.MAX_VALUE;
      deltaCount = Math.min(deltaCount, modules.front_left.getPositionDeltas().length);
      deltaCount = Math.min(deltaCount, modules.front_right.getPositionDeltas().length);
      deltaCount = Math.min(deltaCount, modules.back_left.getPositionDeltas().length);
      deltaCount = Math.min(deltaCount, modules.back_right.getPositionDeltas().length);

      for (int deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++) {
        // Read wheel deltas from each module
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        wheelDeltas[0] = modules.front_left.getPositionDeltas()[deltaIndex];
        wheelDeltas[1] = modules.front_right.getPositionDeltas()[deltaIndex];
        wheelDeltas[2] = modules.back_left.getPositionDeltas()[deltaIndex];
        wheelDeltas[3] = modules.back_right.getPositionDeltas()[deltaIndex];

        // The twist represents the motion of the robot since the last
        // sample in x, y, and theta based only on the modules, without
        // the gyro. The gyro is always disconnected in simulation.
        var twist = kinematics.toTwist2d(wheelDeltas);
        if (gyroInputs.connected) {
          // If the gyro is connected, replace the theta component of the twist
          // with the change in angle since the last sample.
          Rotation2d gyroRotation = gyroInputs.odometryYawPositions[deltaIndex];
          twist =
              new Twist2d(twist.dx, twist.dy, gyroRotation.minus(lastGyroRotation).getRadians());
          lastGyroRotation = gyroRotation;
        }
        // Apply the twist (change since last sample) to the current pose

        rawOdometryPose = rawOdometryPose.exp(twist);

        // fixme: all the modules should have the same timestamp so taking from 0 should be fine
        var timestamp = modules.front_left.getOdometryTimestamps()[deltaIndex];
        poseEstimator.addDriveData(timestamp, twist);
      }

      Logger.recordOutput("SwerveStates/Measured", getModuleStates());

      simulatedAcceleration =
          new Translation2d(
              accelerationFilterX.calculate(
                  (getFieldRelativeVelocities().getTranslation().getX() - lastVel.getX()) / 0.02),
              accelerationFilterY.calculate(
                  (getFieldRelativeVelocities().getTranslation().getY() - lastVel.getY()) / 0.02));

      Logger.recordOutput("Drivetrain/simualedAcceleration", simulatedAcceleration);

      lastVel = getFieldRelativeVelocities().getTranslation();

      field2d.setRobotPose(poseEstimator.getLatestPose());
      SmartDashboard.putData("Localization/field2d", field2d);

      rawOdometryField2d.setRobotPose(rawOdometryPose);
      SmartDashboard.putData("Localization/rawOdometryField2d", rawOdometryField2d);
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds, boolean prioritizeRotation) {
    if (prioritizeRotation) {
      // Calculate module setpoints

      SwerveModuleState[] justRotationSetpointStates =
          kinematics.toSwerveModuleStates(
              new ChassisSpeeds(0.0, 0.0, speeds.omegaRadiansPerSecond));
      SwerveModuleState[] calculatedSetpointStates = kinematics.toSwerveModuleStates(speeds);
      ChassisSpeeds newSpeeds;
      double realMaxSpeed = 0;
      for (SwerveModuleState moduleState : calculatedSetpointStates) {
        realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
      }
      if (realMaxSpeed > config.getRobotMaxLinearVelocity()) {
        /*

        linear cannot exceed the leftover amount after rotation
        Vmax - abs(Vr) = hypot(Vx, Vy)

        preserve xy ratio
        Vyprev      Vy
        -------- = ----
        Vxprev      Vx

        solve system of equations for Vx and Vy

        Vx = Vxprev / hypot(Vxprev, Vyprev) * abs(max(0.0, Vmax - abs(Vr)))

        Vy = Vyprev * Vx / Vxprev

        */

        double vxMetersPerSecond =
            (speeds.vxMetersPerSecond
                    / Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond))
                * Math.abs(
                    Math.max(
                        0.0,
                        config.getRobotMaxLinearVelocity()
                            - Math.abs(justRotationSetpointStates[0].speedMetersPerSecond)));

        // double vxMetersPerSecond =
        //     Math.copySign(
        //         Math.abs(
        //                 Math.max(
        //                     0.0,
        //                     config.getRobotMaxLinearVelocity()
        //                         - Math.abs(justRotationSetpointStates[0].speedMetersPerSecond)))
        //             / Math.sqrt(
        //                 (1.0 + Math.pow(speeds.vyMetersPerSecond / speeds.vxMetersPerSecond,
        // 2.0))),
        //         speeds.vxMetersPerSecond);
        double vyMetersPerSecond =
            speeds.vyMetersPerSecond * vxMetersPerSecond / speeds.vxMetersPerSecond;

        speeds =
            new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        Logger.recordOutput(
            "Drivetrain/leftoverVelocity",
            config.getRobotMaxLinearVelocity()
                - justRotationSetpointStates[0].speedMetersPerSecond);
      }
    }

    Logger.recordOutput("Drivetrain/speedsX", speeds.vxMetersPerSecond);
    Logger.recordOutput("Drivetrain/speedsY", speeds.vyMetersPerSecond);
    Logger.recordOutput(
        "Drivetrain/linearSpeed", Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    Logger.recordOutput("Drivetrain/linearSpeedMax", config.getRobotMaxLinearVelocity());
    Logger.recordOutput("Drivetrain/speedsRot", speeds.omegaRadiansPerSecond);

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, config.getRobotMaxLinearVelocity());

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    // The module returns the optimized state, useful for logging
    optimizedSetpointStates[0] = modules.front_left.runSetpoint(setpointStates[0]);
    optimizedSetpointStates[1] = modules.front_right.runSetpoint(setpointStates[1]);
    optimizedSetpointStates[2] = modules.back_left.runSetpoint(setpointStates[2]);
    optimizedSetpointStates[3] = modules.back_right.runSetpoint(setpointStates[3]);

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds(), false);
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      // each translation forms a triangle from the center of the robot with the appropriate angle
      // for X stance
      headings[i] = config.getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    modules.front_left.runCharacterization(volts);
    modules.front_right.runCharacterization(volts);
    modules.back_left.runCharacterization(volts);
    modules.back_right.runCharacterization(volts);
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage =
        modules.front_left.getCharacterizationVelocity()
            + modules.front_right.getCharacterizationVelocity()
            + modules.back_left.getCharacterizationVelocity()
            + modules.back_right.getCharacterizationVelocity();

    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      modules.front_left.getState(),
      modules.front_right.getState(),
      modules.back_left.getState(),
      modules.back_right.getState()
    };
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  @AutoLogOutput(key = "Robot/FieldRelativeVel")
  public Pose2d getFieldRelativeVelocities() {
    Translation2d translation =
        new Translation2d(
                getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond)
            .rotateBy(getRawOdometryPose().getRotation());
    return new Pose2d(translation, new Rotation2d(getChassisSpeeds().omegaRadiansPerSecond));
  }

  private LinearFilter accelerationFilterX = LinearFilter.movingAverage(3);
  private LinearFilter accelerationFilterY = LinearFilter.movingAverage(3);

  @AutoLogOutput(key = "Robot/FieldRelativeAcceleration")
  public Translation2d getFieldRelativeAccelerations() {
    if (Robot.isReal()) {
      return new Translation2d(gyroInputs.xAcceleration, gyroInputs.yAcceleration)
          .rotateBy(new Rotation2d(-getRawOdometryPose().getRotation().getRadians()));
    } else {
      return simulatedAcceleration;
    }
  }

  public void addVisionEstimate(List<TimestampedVisionUpdate> visionData) {
    for (TimestampedVisionUpdate v : visionData) {
      if (GeometryUtil.isPoseOutsideField(v.pose())) {
        Logger.recordOutput("Vision/ingorePoseBecauseOutOfBounds", true);
        return;
      }
    }
    Logger.recordOutput("Vision/ingorePoseBecauseOutOfBounds", false);
    poseEstimator.addVisionData(visionData);
  }

  /**
   * WARNING - THIS IS THE RAW *ODOMETRY* POSE, THIS DOES NOT ACCOUNT FOR VISION DATA & SHOULD
   * EXCLUSIVELY BE USED FOR LOGGING AND ANALYSIS
   */
  @AutoLogOutput(key = "Localization/RobotPosition_RAW_ODOMETRY")
  private Pose2d getRawOdometryPose() {
    return rawOdometryPose;
  }

  @AutoLogOutput(key = "Localization/RobotPosition")
  public Pose2d getPoseEstimatorPose() {
    return poseEstimator.getLatestPose();
  }

  public Pose2d getFutureEstimatedPose(double sec, String userClassName) {
    Translation2d velocityContribution = getFieldRelativeVelocities().getTranslation().times(sec);
    Translation2d accelerationContribution =
        getFieldRelativeAccelerations().times(Math.pow(sec, 2) * 0.5);
    Pose2d pose =
        new Pose2d(
            getPoseEstimatorPose().getX()
                + velocityContribution.getX()
                + accelerationContribution.getX(),
            getPoseEstimatorPose().getY()
                + velocityContribution.getY()
                + accelerationContribution.getY(),
            new Rotation2d(
                getPoseEstimatorPose().getRotation().getRadians()
                    + gyroInputs.yawVelocityRadPerSec * sec));
    Logger.recordOutput(
        userClassName + "/futureEstimatedPose/velocityContribution", velocityContribution);
    Logger.recordOutput(
        userClassName + "/futureEstimatedPose/accelerationContribution", accelerationContribution);
    Logger.recordOutput(userClassName + "/futureEstimatedPose/pose", pose);
    return pose;
  }

  public Rotation2d getRotation() {
    return getPoseEstimatorPose().getRotation();
  }

  public Rotation2d getRotationGyroOnly() {
    return rawOdometryPose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    this.poseEstimator.resetPose(pose);
    this.rawOdometryPose = pose;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return config.getRobotMaxLinearVelocity();
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return config.getRobotMaxAngularVelocity();
  }

  public double[] getCurrentDrawAmps() {
    double[] current = new double[] {0};
    current = ArrayUtil.concatWithArrayCopy(current, modules.front_left.getCurrentAmps());
    current = ArrayUtil.concatWithArrayCopy(current, modules.front_right.getCurrentAmps());
    current = ArrayUtil.concatWithArrayCopy(current, modules.back_left.getCurrentAmps());
    current = ArrayUtil.concatWithArrayCopy(current, modules.back_right.getCurrentAmps());
    return current;
  }
}
