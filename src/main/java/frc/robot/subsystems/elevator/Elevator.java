// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private static final TunableNumberGroup group = new TunableNumberGroup("Elevator");

  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kD = group.build("kD");
  private static final LoggedTunableNumber kG = group.build("kG");

  private static final LoggedTunableNumber closedLoopMaxVelocityConstraint =
      group.build("defaultClosedLoopMaxVelocityConstraint");
  private static final LoggedTunableNumber closedLoopMaxAccelerationConstraint =
      group.build("defaultClosedLoopMaxAccelerationConstraint");

  private final LoggedTunableNumber tolerance = group.build("toleranceInches", 0.1);

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_SIMBOT) {
      kP.initDefault(1.0);
      kD.initDefault(0.0);
      kG.initDefault(0.0);

      closedLoopMaxVelocityConstraint.initDefault(640.0);
      closedLoopMaxAccelerationConstraint.initDefault(640.0);

    } else if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_MAESTRO) {
      kP.initDefault(12.0);
      kD.initDefault(0.0);
      kG.initDefault(0.0);

      closedLoopMaxVelocityConstraint.initDefault(1000.0);
      closedLoopMaxAccelerationConstraint.initDefault(2000.0);
    }
  }

  private double lastServoActivationTime = 0.0;
  private boolean rightServoActive = false;
  private Rotation2d targetServoAngle = new Rotation2d();

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private Measure<Distance> targetHeight = Units.Meters.zero();

  /** Creates a new ElevatorSubsystem. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    io.setPIDConstraints(
        kP.get(),
        kD.get(),
        kG.get(),
        new Constraints(
            closedLoopMaxVelocityConstraint.get(), closedLoopMaxAccelerationConstraint.get()));
  }

  @Override
  public void periodic() {
    try (var ignored = new ExecutionTiming("Elevator")) {
      io.updateInputs(inputs);
      Logger.processInputs("Elevator", inputs);

      Logger.recordOutput("Elevator/targetHeight", targetHeight.in(Units.Inches));

      // ---- UPDATE TUNABLE NUMBERS
      var hc = hashCode();
      if (kP.hasChanged(hc)
          || kD.hasChanged(hc)
          || kG.hasChanged(hc)
          || closedLoopMaxVelocityConstraint.hasChanged(hc)
          || closedLoopMaxAccelerationConstraint.hasChanged(hc)) {
        io.setPIDConstraints(
            kP.get(),
            kD.get(),
            kG.get(),
            new Constraints(
                closedLoopMaxVelocityConstraint.get(), closedLoopMaxAccelerationConstraint.get()));
      }

      if (Constants.unusedCode) {
        if (Timer.getFPGATimestamp() >= lastServoActivationTime + 2.0) {
          lastServoActivationTime = Timer.getFPGATimestamp();
          if (rightServoActive) {
            io.setLeftServoAngle(targetServoAngle);
            rightServoActive = false;
          } else {
            io.setRightServoAngle(targetServoAngle);
            rightServoActive = true;
          }
        }
      }
    }
  }

  public void resetSubsystem() {
    io.setVoltage(0);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setHeight(Measure<Distance> height) {
    io.setHeight(height);
    targetHeight = height;
  }

  public boolean isAtTarget() {
    return Math.abs(targetHeight.in(Units.Inches) - inputs.heightInches) <= tolerance.get();
  }

  public boolean isAtTarget(Measure<Distance> height) {
    return Math.abs(height.in(Units.Inches) - inputs.heightInches) <= tolerance.get();
  }

  public double getHeightInches() {
    return inputs.heightInches;
  }

  public void resetSensorToHomePosition() {
    io.setSensorPosition(Constants.ElevatorConstants.HOME_POSITION);
  }

  public double getVoltage() {
    return inputs.appliedVolts;
  }

  public double getVelocity() {
    return inputs.velocityInchesPerSecond;
  }

  public void setNeutralMode(NeutralModeValue value) {
    io.setNeutralMode(value);
  }

  public void deployReactionArms() {
    targetServoAngle = Rotation2d.fromDegrees(180.0);
  }
}
