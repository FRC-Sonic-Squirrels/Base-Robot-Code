package frc.robot.autonomous.substates;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autonomous.AutosSubsystems;
import frc.robot.autonomous.ChoreoTrajectoryWithName;
import frc.robot.autonomous.DriveToGamepieceHelper;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.visionGamepiece.ProcessedGamepieceData;
import java.util.function.Supplier;

public class AutoSubstateMachineDriveTranslation extends AutoSubstateMachine {
  private final Translation2d gamepieceTranslation;

  /** Creates a new AutoSubstateMachineDriveTranslation. */
  public AutoSubstateMachineDriveTranslation(
      AutosSubsystems subsystems,
      RobotConfig config,
      boolean useVision,
      Translation2d gamepieceTranslation,
      ChoreoTrajectoryWithName trajToShoot,
      Supplier<ProcessedGamepieceData> closestGamepiece) {
    super(
        "AutoSub " + ChoreoTrajectoryWithName.getName(trajToShoot),
        subsystems,
        config,
        useVision,
        trajToShoot,
        closestGamepiece,
        gamepieceTranslation);

    this.gamepieceTranslation = gamepieceTranslation;

    setInitialState(stateWithName("initFollowPathToGamePiece", this::initFollowPathToGamePiece));
  }

  private StateHandler initFollowPathToGamePiece() {
    intakeCommand = new IntakeGamepiece(intake, endEffector, shooter, arm, elevator);
    intakeCommand.schedule();

    driveToGamepieceHelper = new DriveToGamepieceHelper(led);

    return stateWithName("followPathToGamePiece", this::pickupGamepiece);
  }

  private StateHandler pickupGamepiece() {
    ChassisSpeeds speeds =
        driveToGamepieceHelper.calculateChassisSpeeds(
            gamepieceTranslation, drive.getPoseEstimatorPose(true));

    if (useVisionForGamepiece()) {
      return stateWithName("visionPickupGamepiece", super::visionPickupGamepiece);
    }

    if (!endEffector.noteInEndEffector()) {
      drive.setVelocityOverride(speeds);
      if (driveToGamepieceHelper.isAtTarget()) {
        drive.resetVelocityOverride();
        return super::gamepieceConfirmation;
      }
      return null;
    }
    drive.resetVelocityOverride();
    return stateWithName("prepFollowPathToShooting", super::prepFollowPathToShooting);
  }
}
