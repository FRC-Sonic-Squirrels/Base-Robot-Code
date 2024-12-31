// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Supplier;

public class ShooterSetPivotAngle extends Command {
  /** Creates a new ArmSetAngle. */
  Shooter shooter;

  Supplier<Rotation2d> angleSupplier;

  public ShooterSetPivotAngle(Shooter shooter, Rotation2d angle) {
    this(shooter, () -> angle);
  }

  public ShooterSetPivotAngle(Shooter shooter, Supplier<Rotation2d> angleSupplier) {
    this.shooter = shooter;
    this.angleSupplier = angleSupplier;

    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.setPivotPosition(angleSupplier.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
