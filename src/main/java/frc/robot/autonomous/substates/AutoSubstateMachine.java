// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.substates;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.StateMachine;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.autonomous.AutosSubsystems;
import frc.robot.autonomous.ChoreoHelper;
import frc.robot.autonomous.ChoreoTrajectoryWithName;
import frc.robot.autonomous.DriveToGamepieceHelper;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.visionGamepiece.ProcessedGamepieceData;
import java.util.function.Supplier;

public abstract class AutoSubstateMachine extends StateMachine {
  protected final DrivetrainWrapper drive;
  protected final LED led;
  protected final RobotConfig config;
  protected final ChoreoTrajectoryWithName trajToShoot;
  protected final Supplier<ProcessedGamepieceData> closestGamepiece;
  private final boolean useVision;
  private final boolean ploppedGamepiece;
  protected ChoreoHelper choreoHelper;
  protected DriveToGamepieceHelper driveToGamepieceHelper;
  public Command score;
  protected IntakeGamepiece intakeCommand;
  protected Translation2d gamepieceTranslation;
  private Translation2d lastSeenGamepiece;

  private static final LoggerGroup logGroupGPVision = LoggerGroup.build("AutoGamepieceVision");
  private static final LoggerEntry.Text log_GPVisionStatus = logGroupGPVision.buildString("status");
  private static final LoggerEntry.Decimal log_GPVisionDetectedGamepieceDistanceFromExpectedTarget =
      logGroupGPVision.buildDecimal("detectedGamepieceDistanceFromExpectedTarget");
  private static final LoggerEntry.Decimal log_GPVisionDistanceFromRobot =
      logGroupGPVision.buildDecimal("distanceFromRobot");
  private static final LoggerEntry.Bool log_GPVisionWithinDistanceToRobot =
      logGroupGPVision.buildBoolean("withinDistanceToRobot");
  private static final LoggerEntry.Bool log_GPVisionDetectedGamepieceWithinExpectedRange =
      logGroupGPVision.buildBoolean("detectedGamepieceWithinExpectedRange");

  private static final TunableNumberGroup groupTunable =
      new TunableNumberGroup("AutoSubstateMachine");
  private static final LoggedTunableNumber distToBeginDriveToGamepiece =
      groupTunable.build("distToBeginDriveToGamepiece", 2.0);
  private static final LoggedTunableNumber confirmationTime =
      groupTunable.build("confirmationTime", 0.4);
  protected static final LoggedTunableNumber slowDownFactor =
      groupTunable.build("slowDownFactor", 1.0);
  private static final LoggedTunableNumber distFromExpectedToAcceptVisionGamepiece =
      groupTunable.build("distFromExpectedToAcceptVisionGamepiece", 1.0);
  protected static final LoggedTunableNumber minVelToPause =
      groupTunable.build("minVelToPause", 2.0);

  /** Creates a new AutoSubstateMachine. */
  protected AutoSubstateMachine(
      String name,
      AutosSubsystems subsystems,
      RobotConfig config,
      boolean useVision,
      boolean ploppedGamepiece,
      ChoreoTrajectoryWithName trajToShoot,
      Supplier<ProcessedGamepieceData> closestGamepiece,
      Translation2d gamepieceTranslation) {
    super(name);

    this.drive = subsystems.drivetrain();
    this.led = subsystems.led();
    this.useVision = useVision;
    this.ploppedGamepiece = ploppedGamepiece;
    this.config = config;
    this.trajToShoot = trajToShoot;
    this.closestGamepiece = closestGamepiece;
    this.gamepieceTranslation = gamepieceTranslation;
  }

  protected StateHandler visionPickupGamepiece() {
    led.setBaseRobotState(BaseRobotState.AUTO_GAMEPIECE_PICKUP);
    ProcessedGamepieceData gamepieceData = closestGamepiece.get();
    if (gamepieceData != null
        && useVisionForGamepiece()
        && gamepieceData.getDistance(drive.getPoseEstimatorPose(true)).in(Units.Inches) > 35.0) {
      lastSeenGamepiece = gamepieceData.globalPose.getTranslation();
    }

    ChassisSpeeds speeds =
        driveToGamepieceHelper.calculateChassisSpeeds(
            lastSeenGamepiece, drive.getPoseEstimatorPose(true));

    if (speeds != null) drive.setVelocityOverride(speeds);

    // TODO: change this condition to whether gamepiece is posessed:
    if (false) {
      if (driveToGamepieceHelper.isAtTarget()) {
        drive.resetVelocityOverride();
        led.setBaseRobotState(BaseRobotState.GAMEPIECE_STATUS);
        return setStopped();
      }
      return null;
    }
    drive.resetVelocityOverride();
    led.setBaseRobotState(BaseRobotState.GAMEPIECE_STATUS);
    return stateWithName("prepFollowPathToShooting", this::prepFollowPathToShooting);
  }

  protected StateHandler prepFollowPathToShooting() {
    intakeCommand.cancel();

    // TODO: Change to scoring command
    score = Commands.runOnce(null, null);

    spawnCommand(
        score,
        (command) -> {
          drive.resetVelocityOverride();
          return setDone();
        });

    if (trajToShoot != null) {
      choreoHelper =
          new ChoreoHelper(
              timeFromStart(),
              drive.getPoseEstimatorPose(true),
              trajToShoot.rescale(slowDownFactor.get()),
              config.getDriveBaseRadius() / 2,
              minVelToPause.get(),
              config.getAutoTranslationPidController(),
              config.getAutoTranslationPidController(),
              config.getAutoThetaPidController());
    }

    return stateWithName("followPathToShooter", this::followPathToShooting);
  }

  private StateHandler followPathToShooting() {
    if (choreoHelper != null) {
      var result =
          choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(true), timeFromStart());
      drive.setVelocityOverride(result.chassisSpeeds());
    } else {
      drive.resetVelocityOverride();
    }

    return null;
  }

  protected StateHandler gamepieceConfirmation() {
    // TODO: change condition to if gamepiece is posessed.
    if (false) {
      return stateWithName("prepFollowPathToShooting", this::prepFollowPathToShooting);
    }
    if (timeFromStartOfState() >= confirmationTime.get()) {
      return setStopped();
    }
    return null;
  }

  protected boolean useVisionForGamepiece() {
    if (!useVision) {
      log_GPVisionStatus.info("disabled");
      log_GPVisionDetectedGamepieceDistanceFromExpectedTarget.info(-1);
      log_GPVisionDistanceFromRobot.info(-1);
      return false;
    }

    ProcessedGamepieceData gamepieceData = closestGamepiece.get();
    if (gamepieceData == null) {
      log_GPVisionStatus.info("data null");
      log_GPVisionDetectedGamepieceDistanceFromExpectedTarget.info(-1);
      log_GPVisionDistanceFromRobot.info(-1);
      return false;
    }

    Pose2d robotPose = drive.getPoseEstimatorPose(true);
    double distanceToRobot = gamepieceData.getDistance(robotPose).in(Units.Meters);
    double distanceToTarget = gamepieceData.getDistance(gamepieceTranslation).in(Units.Meters);

    var withInBeginDriveDistance = distanceToRobot <= distToBeginDriveToGamepiece.get();
    var withInExpectedTargetDistance =
        distanceToTarget <= distFromExpectedToAcceptVisionGamepiece.get();

    log_GPVisionStatus.info(
        withInBeginDriveDistance && withInExpectedTargetDistance ? "SUCCESSFUL" : "FAILED_CHECKS");

    log_GPVisionDetectedGamepieceDistanceFromExpectedTarget.info(distanceToTarget);
    log_GPVisionDistanceFromRobot.info(distanceToRobot);
    log_GPVisionWithinDistanceToRobot.info(withInBeginDriveDistance);
    log_GPVisionDetectedGamepieceWithinExpectedRange.info(withInExpectedTargetDistance);

    Pose2d flippedPose = AllianceFlipUtil.flipPoseForAlliance(robotPose);
    Translation2d flippedGamepieceTranslation =
        AllianceFlipUtil.flipTranslationForAlliance(gamepieceTranslation);

    boolean poseIsPastWingLine = flippedPose.getX() > 5.572144031524658;
    boolean gamepieceIsPastWingLine = flippedGamepieceTranslation.getX() > 5.572144031524658;
    boolean sameSideAsGamepiece = poseIsPastWingLine == gamepieceIsPastWingLine;

    return
    // withInBeginDriveDistance
    (withInExpectedTargetDistance || ploppedGamepiece) && sameSideAsGamepiece;
  }
}
