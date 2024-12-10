package frc.robot.autonomous;

import frc.robot.subsystems.LED;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.visionGamepiece.VisionGamepiece;

public record AutosSubsystems(
    DrivetrainWrapper drivetrain, VisionGamepiece visionGamepiece, LED led) {}
