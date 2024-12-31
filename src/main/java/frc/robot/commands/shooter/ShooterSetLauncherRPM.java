// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class ShooterSetLauncherRPM extends Command {

  private Shooter shooter;
  private DoubleSupplier RPM;

  /** Creates a new IntakeSetRPM. */
  public ShooterSetLauncherRPM(Shooter shooter, DoubleSupplier RPM) {

    this.shooter = shooter;
    this.RPM = RPM;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  public ShooterSetLauncherRPM(Shooter shooter, double RPM) {
    this(shooter, () -> RPM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setLauncherRPM(RPM.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setLauncherPercentOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
