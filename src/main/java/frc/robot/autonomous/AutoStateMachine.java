// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import frc.lib.team2930.StateMachine;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import org.littletonrobotics.junction.Logger;

public class AutoStateMachine extends StateMachine {
  ScoreSpeaker scoreSpeaker;
  private final DrivetrainWrapper drive;
  private final Shooter shooter;
  private final StateMachine[] subStates;
  private int currentSubState;
  private double initShootDeadline;

  /** Creates a new AutoSubstateMachine. */
  public AutoStateMachine(
      DrivetrainWrapper drive,
      Shooter shooter,
      StateMachine[] subStates,
      double initShootDeadline) {
    this.drive = drive;
    this.shooter = shooter;
    this.subStates = subStates;
    this.initShootDeadline = initShootDeadline;

    setInitialState(this::makeInitialShot);
  }

  private StateHandler makeInitialShot() {
    scoreSpeaker = new ScoreSpeaker(drive, shooter, () -> true, initShootDeadline);
    scoreSpeaker.schedule();
    Logger.recordOutput("Autonomous/CommandScheduled", true);

    return this::nextSubState;
  }

  private StateHandler nextSubState() {
    if (currentSubState >= subStates.length) {
      return setDone();
    }

    return suspendForSubStateMachine(
        subStates[currentSubState++], subStateMachine -> AutoStateMachine.this::nextSubState);
  }
}
