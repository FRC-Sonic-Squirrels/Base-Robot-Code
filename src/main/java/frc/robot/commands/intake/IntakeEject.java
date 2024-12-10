// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeEject extends Command {

  /** Creates a new IntakeEject. */
  public IntakeEject() {
    // TODO: Add subsystems
    // TODO: Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Run intake/indexing in reverse
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO: Stop intake/indexing
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
