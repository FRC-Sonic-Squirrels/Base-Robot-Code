// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;

public class Elevator extends SubsystemBase {
  public static final String ROOT_TABLE = "Elevator";

  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  private static final LoggerGroup logInputs = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_heightInches =
      logInputs.buildDecimal("HeightInches");
  private static final LoggerEntry.Decimal logInputs_velocityInchesPerSecond =
      logInputs.buildDecimal("VelocityInchesPerSecond");
  private static final LoggerEntry.Decimal logInputs_appliedVolts =
      logInputs.buildDecimal("AppliedVolts");
  private static final LoggerEntry.Decimal logInputs_currentAmps =
      logInputs.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal logInputs_tempCelsius =
      logInputs.buildDecimal("TempCelsius");

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Decimal logTargetHeight = logGroup.buildDecimal("targetHeight");

  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);

  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kD = group.build("kD");
  private static final LoggedTunableNumber kG = group.build("kG");

  private static final LoggedTunableNumber closedLoopMaxVelocityConstraint =
      group.build("defaultClosedLoopMaxVelocityConstraint");
  private static final LoggedTunableNumber closedLoopMaxAccelerationConstraint =
      group.build("defaultClosedLoopMaxAccelerationConstraint");

  private final LoggedTunableNumber tolerance = group.build("toleranceInches", 0.1);

  private static final Constraints motionMagicDefaultConstraints;

  private Constraints currentMotionMagicConstraints = new Constraints(0.0, 0.0);

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_SIMBOT) {
      kP.initDefault(0.01);
      kD.initDefault(0.0);
      kG.initDefault(1.84255);

      closedLoopMaxVelocityConstraint.initDefault(640.0);
      closedLoopMaxAccelerationConstraint.initDefault(640.0);
      motionMagicDefaultConstraints = new Constraints(640.0, 640.0);

    } else if (Constants.RobotMode.getRobot() == RobotType.ROBOT_COMPETITION) {
      kP.initDefault(12.0);
      kD.initDefault(0.0);
      kG.initDefault(0.0);

      closedLoopMaxVelocityConstraint.initDefault(1000.0);
      closedLoopMaxAccelerationConstraint.initDefault(2000.0);

      motionMagicDefaultConstraints = new Constraints(1000.0, 2000.0);
    } else {
      motionMagicDefaultConstraints = new Constraints(640.0, 640.0);
    }
  }

  private final ElevatorIO io;
  private final ElevatorIO.Inputs inputs = new ElevatorIO.Inputs(logInputs);
  private Measure<Distance> targetHeight = Units.Meters.zero();

  /** Creates a new ElevatorSubsystem. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    setConstants();
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      io.updateInputs(inputs);
      logInputs_heightInches.info(inputs.heightInches);
      logInputs_velocityInchesPerSecond.info(inputs.velocityInchesPerSecond);
      logInputs_appliedVolts.info(inputs.appliedVolts);
      logInputs_currentAmps.info(inputs.currentAmps);
      logInputs_tempCelsius.info(inputs.tempCelsius);

      // ---- UPDATE TUNABLE NUMBERS
      var hc = hashCode();
      if (kP.hasChanged(hc)
          || kD.hasChanged(hc)
          || kG.hasChanged(hc)
          || closedLoopMaxVelocityConstraint.hasChanged(hc)
          || closedLoopMaxAccelerationConstraint.hasChanged(hc)) {
        setConstants();
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
    logTargetHeight.info(targetHeight.in(Units.Inches));
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

  public boolean setNeutralMode(NeutralModeValue value) {
    return io.setNeutralMode(value);
  }

  public void setConstants() {
    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
    mmConfigs.MotionMagicAcceleration = closedLoopMaxAccelerationConstraint.get();
    mmConfigs.MotionMagicCruiseVelocity = closedLoopMaxVelocityConstraint.get();
    io.setClosedLoopConstants(kP.get(), kD.get(), kG.get(), mmConfigs);
  }

  public Constraints getDefaultMotionMagicConstraints() {
    return motionMagicDefaultConstraints;
  }

  public Constraints getCurrentMotionMagicConstraints() {
    return currentMotionMagicConstraints;
  }
}
