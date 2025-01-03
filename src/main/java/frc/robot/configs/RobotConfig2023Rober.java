package frc.robot.configs;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.lib.constants.SwerveModuleConstants;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIOTalonFX;
import frc.robot.subsystems.swerve.SwerveModules;
import frc.robot.subsystems.vision.VisionModuleConfiguration;

public class RobotConfig2023Rober extends RobotConfig {

  // FIX ME: BUY PHOENIX PRO
  private static final boolean PHOENIX_PRO_LICENSE = false;

  // ------------ SWERVE ---------------------
  private static final Measure<Distance> WHEEL_RADIUS = Units.Inches.of(2.0);

  // ------ SWERVE MODULE CONFIGURATIONS: CANID + OFFSET + INVERTS --------------
  // 0
  private static final IndividualSwerveModuleConfig FRONT_LEFT_MODULE_CONFIG =
      IndividualSwerveModuleConfig.configureWithDefaultInverts(
          1, 11, 21, Rotation2d.fromDegrees(221.4));
  // 1
  private static final IndividualSwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG =
      IndividualSwerveModuleConfig.configureWithDefaultInverts(
          2, 12, 22, Rotation2d.fromDegrees(190.6));

  // WARNING: Rober has weird a CAN ID order, this should not be copied in the future
  // 2
  private static final IndividualSwerveModuleConfig BACK_LEFT_MODULE_CONFIG =
      new IndividualSwerveModuleConfig(
          4,
          14,
          24,
          Rotation2d.fromDegrees(179.2),
          SwerveModuleConstants.MK4I.DEFAULT_DRIVE_MOTOR_INVERT,
          InvertedValue.CounterClockwise_Positive);
  // 3
  private static final IndividualSwerveModuleConfig BACK_RIGHT_MODULE_CONFIG =
      new IndividualSwerveModuleConfig(
          3,
          13,
          23,
          Rotation2d.fromDegrees(311.9),
          InvertedValue.CounterClockwise_Positive,
          SwerveModuleConstants.MK4I.DEFAULT_STEER_MOTOR_INVERT);

  // -------- SWERVE CURRENT LIMITS ---------
  private static final CurrentLimitsConfigs DRIVE_TALON_CURRENT_LIMIT_CONFIGS =
      new CurrentLimitsConfigs()
          .withSupplyCurrentLimit(35)
          .withSupplyCurrentThreshold(60)
          .withSupplyTimeThreshold(0.1)
          .withSupplyCurrentLimitEnable(true);

  private static final CurrentLimitsConfigs STEER_TALON_CURRENT_LIMIT_CONFIGS =
      new CurrentLimitsConfigs()
          .withSupplyCurrentLimit(25)
          .withSupplyCurrentThreshold(40)
          .withSupplyTimeThreshold(0.1)
          .withSupplyCurrentLimitEnable(true);

  // --------- SWERVE GEAR RATIO ---------
  private static final double SWERVE_DRIVE_GEAR_RATIO =
      SwerveModuleConstants.MK4I.LEVEL_2_GEARING_DRIVE_GEAR_RATIO;
  private static final double SWERVE_STEER_GEAR_RATIO =
      SwerveModuleConstants.MK4I.GEARING_TURN_GEAR_RATIO;

  // ---------- SWERVE STEERING MOTOR PID CONSTANTS -----------
  // FIXE: RN copied from Mechanical advantage (6328) 2023 codebase. Should learn to tune
  // ourselves
  private static final LoggedTunableNumber ANGLE_KP = group.build("ANGLE_KP", 400.0);
  private static final LoggedTunableNumber ANGLE_KD = group.build("ANGLE_KD", 2.0);

  // ---------- SWERVE DRIVE MOTOR PID + KS + KV + KA CONSTANTS -------------
  private static final LoggedTunableNumber DRIVE_KP = group.build("DRIVE_KP", 0.1);
  private static final LoggedTunableNumber DRIVE_KD = group.build("DRIVE_KD", 0.0);

  private static final LoggedTunableNumber DRIVE_KS = group.build("DRIVE_KS", 0.189);
  private static final LoggedTunableNumber DRIVE_KV = group.build("DRIVE_KV", 0.128);
  private static final LoggedTunableNumber DRIVE_KA = group.build("DRIVE_KA", 0.0);

  // -------- GYRO CAN ID ---------
  private static final int GYRO_CAN_ID = 15;

  // -------- GYRO OFFSETS --------

  private static final double GYRO_MOUNTING_PITCH = 0.0;

  private static final double GYRO_MOUNTING_ROLL = 0.0;

  private static final double GYRO_MOUNTING_YAW = 0.0;

  // -------- CAN BUS NAME -----------
  private static final String CAN_BUS_NAME = "CANivore";

  // -------- ROBOT DIMENSIONS -----------
  // front to back
  private static final Measure<Distance> TRACK_WIDTH_X = Units.Inches.of(25);
  // left to right
  private static final Measure<Distance> TRACK_WIDTH_Y = Units.Inches.of(23);

  // ------- ROBOT MAX SPEED --------
  private static final double MAX_VELOCITY_METERS_PER_SECOND = 4.78;
  private static final double MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05;

  // ------- AUTONOMOUS CONSTANTS -------
  private static final LoggedTunableNumber AUTO_MAX_SPEED_METERS_PER_SECOND =
      group.build("AUTO_MAX_SPEED", 2.0);
  private static final LoggedTunableNumber AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED =
      group.build("AUTO_MAX_ACCEL", 2.0);

  private static final LoggedTunableNumber AUTO_TRANSLATION_KP =
      group.build("AUTO_TRANSLATION_KP", 6.0);
  private static final LoggedTunableNumber AUTO_TRANSLATION_KI =
      group.build("AUTO_TRANSLATION_KI", 0.0);
  private static final LoggedTunableNumber AUTO_TRANSLATION_KD =
      group.build("AUTO_TRANSLATION_KD", 0.0);
  private static final LoggedTunableNumber AUTO_THETA_KP = group.build("AUTO_THETA_KP", 4.9);
  private static final LoggedTunableNumber AUTO_THETA_KI = group.build("AUTO_THETA_KI", 0.0);
  private static final LoggedTunableNumber AUTO_THETA_KD = group.build("AUTO_THETA_KD", 0.0);

  // ---- VISION CAMERA TRANSFORM3d's -------

  private static final Transform3d FRONT_LEFT_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.Inches.of(-0.51).in(Units.Meters),
              Units.Inches.of(10.2).in(Units.Meters),
              Units.Inches.of(22.0).in(Units.Meters)),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(30.0)));

  private static final Transform3d FRONT_RIGHT_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.Inches.of(-0.51).in(Units.Meters),
              Units.Inches.of(-10.2).in(Units.Meters),
              Units.Inches.of(22.0).in(Units.Meters)),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(-30.0)));

  private static final Transform3d BACK_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.Inches.of(-2.7).in(Units.Meters),
              Units.Inches.of(0.0).in(Units.Meters),
              Units.Inches.of(33.42).in(Units.Meters)),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(10.0), Math.toRadians(180.0)));

  public static final String FRONT_LEFT_CAMERA_NAME = "LeftCamera";
  public static final String FRONT_RIGHT_CAMERA_NAME = "RightCamera";
  public static final String BACK_CAMERA_NAME = "Arducam_OV9281_Camera_4";

  public static final AprilTagFields APRIL_TAG_FIELD = AprilTagFields.k2024Crescendo;

  /*
   *
   *
   * FUNCTIONS RETURNING CONSTANTS:
   *
   * YOU MUST CHANGED 2 METHODS:
   * getSwerveModuleObjects()
   * getVisionModuleObjects()
   *
   */

  // FIXME: define your swerve module objects here
  @Override
  public SwerveModules getSwerveModuleObjects() {
    var front_left = new SwerveModuleIOTalonFX(this, FRONT_LEFT_MODULE_CONFIG);
    var front_right = new SwerveModuleIOTalonFX(this, FRONT_RIGHT_MODULE_CONFIG);
    var back_left = new SwerveModuleIOTalonFX(this, BACK_LEFT_MODULE_CONFIG);
    var back_right = new SwerveModuleIOTalonFX(this, BACK_RIGHT_MODULE_CONFIG);

    return new SwerveModules(
        new SwerveModule(0, this, front_left),
        new SwerveModule(1, this, front_right),
        new SwerveModule(2, this, back_left),
        new SwerveModule(3, this, back_right));
  }

  @Override
  public SwerveModules getReplaySwerveModuleObjects() {
    return new SwerveModules(
        new SwerveModule(0, this, new SwerveModuleIO.Fake()),
        new SwerveModule(1, this, new SwerveModuleIO.Fake()),
        new SwerveModule(2, this, new SwerveModuleIO.Fake()),
        new SwerveModule(3, this, new SwerveModuleIO.Fake()));
  }

  // FIXME: define vision modules here
  @Override
  public VisionModuleConfiguration[] getVisionModuleObjects() {
    return new VisionModuleConfiguration[] {
      VisionModuleConfiguration.build(FRONT_LEFT_CAMERA_NAME, FRONT_LEFT_ROBOT_TO_CAMERA),
      VisionModuleConfiguration.build(FRONT_RIGHT_CAMERA_NAME, FRONT_RIGHT_ROBOT_TO_CAMERA)
    };
  }

  @Override
  public VisionModuleConfiguration[] getReplayVisionModules() {
    return new VisionModuleConfiguration[] {
      VisionModuleConfiguration.buildReplayStub(FRONT_LEFT_CAMERA_NAME, FRONT_LEFT_ROBOT_TO_CAMERA),
      VisionModuleConfiguration.buildReplayStub(
          FRONT_RIGHT_CAMERA_NAME, FRONT_RIGHT_ROBOT_TO_CAMERA)
    };
  }

  @Override
  public AprilTagFieldLayout getAprilTagFieldLayout() {
    return APRIL_TAG_FIELD.loadAprilTagLayoutField();
  }

  @Override
  public boolean getPhoenix6Licensed() {
    return PHOENIX_PRO_LICENSE;
  }

  @Override
  public IndividualSwerveModuleConfig[] getIndividualModuleConfigurations() {
    return new IndividualSwerveModuleConfig[] {
      FRONT_LEFT_MODULE_CONFIG,
      FRONT_RIGHT_MODULE_CONFIG,
      BACK_LEFT_MODULE_CONFIG,
      BACK_RIGHT_MODULE_CONFIG
    };
  }

  @Override
  public int getGyroCANID() {
    return GYRO_CAN_ID;
  }

  @Override
  public Measure<Distance> getTrackWidth_Y() {
    return TRACK_WIDTH_Y;
  }

  @Override
  public Measure<Distance> getTrackWidth_X() {
    return TRACK_WIDTH_X;
  }

  @Override
  public double getRobotMaxLinearVelocity() {
    return MAX_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public double getRobotMaxCoastVelocity() {
    return MAX_COAST_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public String getCANBusName() {
    return CAN_BUS_NAME;
  }

  @Override
  public LoggedTunableNumber getDriveKS() {
    return DRIVE_KS;
  }

  @Override
  public LoggedTunableNumber getDriveKV() {
    return DRIVE_KV;
  }

  @Override
  public LoggedTunableNumber getDriveKA() {
    return DRIVE_KA;
  }

  @Override
  public LoggedTunableNumber getDriveKP() {
    return DRIVE_KP;
  }

  @Override
  public LoggedTunableNumber getDriveKD() {
    return DRIVE_KD;
  }

  @Override
  public LoggedTunableNumber getAngleKP() {
    return ANGLE_KP;
  }

  @Override
  public LoggedTunableNumber getAngleKD() {
    return ANGLE_KD;
  }

  @Override
  public LoggedTunableNumber getAutoMaxSpeed() {
    return AUTO_MAX_SPEED_METERS_PER_SECOND;
  }

  @Override
  public LoggedTunableNumber getAutoMaxAcceleration() {
    return AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
  }

  @Override
  public PIDController getAutoTranslationPidController() {
    return new PIDController(
        AUTO_TRANSLATION_KP.get(), AUTO_TRANSLATION_KI.get(), AUTO_TRANSLATION_KD.get());
  }

  @Override
  public PIDController getAutoThetaPidController() {
    return new PIDController(AUTO_THETA_KP.get(), AUTO_THETA_KI.get(), AUTO_THETA_KD.get());
  }

  @Override
  public Measure<Distance> getWheelRadius() {
    return WHEEL_RADIUS;
  }

  @Override
  public double getSwerveModuleDriveGearRatio() {
    return SWERVE_DRIVE_GEAR_RATIO;
  }

  @Override
  public double getSwerveModuleTurnGearRatio() {
    return SWERVE_STEER_GEAR_RATIO;
  }

  @Override
  public CurrentLimitsConfigs getDriveTalonCurrentLimitConfig() {
    return DRIVE_TALON_CURRENT_LIMIT_CONFIGS;
  }

  @Override
  public CurrentLimitsConfigs getSteerTalonCurrentLimitConfig() {
    return STEER_TALON_CURRENT_LIMIT_CONFIGS;
  }

  @Override
  public double getGyroMountingPitch() {
    return GYRO_MOUNTING_PITCH;
  }

  @Override
  public double getGyroMountingRoll() {
    return GYRO_MOUNTING_ROLL;
  }

  @Override
  public double getGyroMountingYaw() {
    return GYRO_MOUNTING_YAW;
  }
}
