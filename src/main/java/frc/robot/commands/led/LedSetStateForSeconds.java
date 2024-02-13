// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.robotStates;

public class LedSetStateForSeconds extends Command {
  LED led;
  robotStates state;
  int seconds = 0;

  /** Creates a new LedSetStateForSeconds. */
  public LedSetStateForSeconds(LED led, robotStates state, int seconds) {
    this.led = led;
    this.state = state;
    this.seconds = seconds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.setRobotState(state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // FIXME: change argument to NOTHING after merge
    led.setRobotState(null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
