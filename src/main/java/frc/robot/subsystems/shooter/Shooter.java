// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.LauncherConstants;
import frc.robot.Constants.ShooterConstants.PivotConstants;

public class Shooter extends SubsystemBase {
  private static final ExecutionTiming timing = new ExecutionTiming(ShooterConstants.ROOT_TABLE);

  // Logging
  private static final LoggerGroup logGroup = LoggerGroup.build(ShooterConstants.ROOT_TABLE);

  // Pivot Logging
  private static final LoggerEntry.Decimal logInputs_pivotPosition =
      logGroup.buildDecimal("Pivot/Position");
  private static final LoggerEntry.Decimal logInputs_pivotVelocityRadsPerSec =
      logGroup.buildDecimal("Pivot/VelocityRadsPerSec");
  private static final LoggerEntry.Decimal logInputs_pivotAppliedVolts =
      logGroup.buildDecimal("Pivot/AppliedVolts");
  private static final LoggerEntry.Decimal logInputs_pivotCurrentAmps =
      logGroup.buildDecimal("Pivot/CurrentAmps");

  private static final LoggerEntry.Decimal logPivotTargetAngleDegrees =
      logGroup.buildDecimal("Pivot/TargetAngleDegrees");
  private static final LoggerEntry.EnumValue<ControlMode> logPivotControlMode =
      logGroup.buildEnum("Pivot/ControlMode");

  // Launcher Logging
  private static final LoggerEntry.DecimalArray logInputs_launcherRPM =
      logGroup.buildDecimalArray("Launcher/RPM");
  private static final LoggerEntry.DecimalArray logInputs_launcherAppliedVolts =
      logGroup.buildDecimalArray("Launcher/AppliedVolts");
  private static final LoggerEntry.DecimalArray logInputs_launcherCurrentAmps =
      logGroup.buildDecimalArray("Launcher/CurrentAmps");
  private static final LoggerEntry.DecimalArray logInputs_tempsCelcius =
      logGroup.buildDecimalArray("Launcher/TempsCelcius");

  private static final LoggerEntry.DecimalArray logLauncherTargetRPM =
      logGroup.buildDecimalArray("Launcher/TargetRPM");
  private static final LoggerEntry.EnumValue<ControlMode> logLauncherControlMode =
      logGroup.buildEnum("Launcher/ControlMode");

  // Tunable Numbers
  public static final TunableNumberGroup group =
      new TunableNumberGroup(ShooterConstants.ROOT_TABLE);

  // Pivot Tunable Numbers TODO: Tune
  private static final TunableNumberGroup groupPivot = group.subgroup(PivotConstants.ROOT_TABLE);
  private static final LoggedTunableNumber pivotkP = groupPivot.build("kP");
  private static final LoggedTunableNumber pivotkD = groupPivot.build("kD");
  private static final LoggedTunableNumber pivotkG = groupPivot.build("kG");
  private static final LoggedTunableNumber pivotMaxVelocityConfig =
      groupPivot.build("MaxVelocityConstraint");
  private static final LoggedTunableNumber pivotTargetAccelerationConfig =
      groupPivot.build("MaxAccelerationConstraint");
  private static final LoggedTunableNumber pivotToleranceDegrees =
      groupPivot.build("pivotToleranceDegrees", 0.5);

  // Launcher Tunable Numbers TODO: Tune
  private static final TunableNumberGroup groupLauncher =
      group.subgroup(LauncherConstants.ROOT_TABLE);
  private static final LoggedTunableNumber launcherkS = groupLauncher.build("kS");
  private static final LoggedTunableNumber launcherkP = groupLauncher.build("kP");
  private static final LoggedTunableNumber launcherkV = groupLauncher.build("kV");
  private static final LoggedTunableNumber launcherTargetAccelerationConfig =
      groupLauncher.build("TargetAccelerationConfig");
  private static final LoggedTunableNumber launcherToleranceRPM =
      groupLauncher.build("launcherToleranceRPM", 150);

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_RETIRED_MAESTRO) {

      pivotkP.initDefault(800.0);
      pivotkD.initDefault(0.0);
      pivotkG.initDefault(0.0);
      pivotMaxVelocityConfig.initDefault(10.0);
      pivotTargetAccelerationConfig.initDefault(5.0);

      launcherkS.initDefault(0.24);
      launcherkP.initDefault(0.4);
      launcherkV.initDefault(0.072);
      launcherTargetAccelerationConfig.initDefault(1000.0);

    } else if (Constants.RobotMode.isSimBot()) {

      pivotkP.initDefault(15.0);
      pivotkD.initDefault(0.0);
      pivotkG.initDefault(0.0);
      pivotMaxVelocityConfig.initDefault(10.0);
      pivotTargetAccelerationConfig.initDefault(10.0);

      launcherkS.initDefault(0.0);
      launcherkP.initDefault(0.05);
      launcherkV.initDefault(1);
      launcherTargetAccelerationConfig.initDefault(0.0);
    }
  }

  private final ShooterIO io;
  private final ShooterIO.Inputs inputs = new ShooterIO.Inputs(logGroup);

  private Rotation2d targetPivotPosition = Constants.zeroRotation2d;
  private double rollerTargetRPM = 0.0;

  private ControlMode launcherControlMode = ControlMode.OPEN_LOOP;
  private ControlMode pivotControlMode = ControlMode.OPEN_LOOP;

  /** Creates a new ShooterSubsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;

    setLauncherClosedLoopConstants();

    setPivotClosedLoopConstants();

    io.setLauncherVoltage(0.0);
    io.setPivotVoltage(0.0);
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      io.updateInputs(inputs);

      // Pivot logging
      logInputs_pivotPosition.info(inputs.pivotPosition);
      logInputs_pivotVelocityRadsPerSec.info(inputs.pivotVelocityDegreesPerSec);
      logInputs_pivotAppliedVolts.info(inputs.pivotAppliedVolts);
      logInputs_pivotCurrentAmps.info(inputs.pivotCurrentAmps);
      logPivotControlMode.info(pivotControlMode);

      // Launcher logging
      logInputs_launcherRPM.info(inputs.launcherRPM);
      logInputs_launcherAppliedVolts.info(inputs.launcherAppliedVolts);
      logInputs_launcherCurrentAmps.info(inputs.launcherCurrentAmps);
      logInputs_tempsCelcius.info(inputs.tempsCelcius);
      logLauncherControlMode.info(launcherControlMode);

      // Update constants
      var hc = hashCode();
      if (launcherkS.hasChanged(hc)
          || launcherkP.hasChanged(hc)
          || launcherkV.hasChanged(hc)
          || launcherTargetAccelerationConfig.hasChanged(hc)) {
        setLauncherClosedLoopConstants();
      }

      if (pivotkP.hasChanged(hc)
          || pivotkD.hasChanged(hc)
          || pivotkG.hasChanged(hc)
          || pivotMaxVelocityConfig.hasChanged(hc)
          || pivotTargetAccelerationConfig.hasChanged(hc)) {

        setPivotClosedLoopConstants();
      }
    }
  }

  // Launcher setters

  public void setLauncherPercentOut(double percent) {
    io.setLauncherVoltage(percent * Constants.MAX_VOLTAGE);
    launcherControlMode = ControlMode.OPEN_LOOP;
  }

  public void setLauncherRPM(double rpm) {
    io.setLauncherRPM(rpm);
    rollerTargetRPM = rpm;
    logLauncherTargetRPM.info(rpm);
    launcherControlMode = ControlMode.CLOSED_LOOP;
  }

  public void setLauncherClosedLoopConstants() {
    io.setLauncherClosedLoopConstants(
        launcherkP.get(),
        launcherkV.get(),
        launcherkS.get(),
        launcherTargetAccelerationConfig.get());
  }

  // Launcher getters

  public double getRPM() {
    return inputs.launcherRPM;
  }

  public boolean isAtTargetRPM() {
    return isAtTargetRPM(rollerTargetRPM);
  }

  public boolean isAtTargetRPM(double rpm) {
    return inputs.launcherRPM > rpm - launcherToleranceRPM.get();
  }

  // Pivot setters

  public void setPivotVoltage(double volts) {
    io.setPivotVoltage(volts);
    pivotControlMode = ControlMode.OPEN_LOOP;
  }

  public void setPivotPosition(Rotation2d rot) {
    io.setPivotPosition(rot);
    targetPivotPosition = rot;
    logPivotTargetAngleDegrees.info(rot);
    pivotControlMode = ControlMode.OPEN_LOOP;
  }

  public void setPivotClosedLoopConstants() {
    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();

    mmConfigs.MotionMagicAcceleration = pivotTargetAccelerationConfig.get();
    mmConfigs.MotionMagicCruiseVelocity = pivotMaxVelocityConfig.get();

    io.setPivotClosedLoopConstants(pivotkP.get(), pivotkD.get(), pivotkG.get(), mmConfigs);
  }

  public void pivotResetHomePosition() {
    io.resetPivotSensorPosition(Constants.ShooterConstants.PivotConstants.HOME_POSITION);
  }

  public boolean setNeutralMode(NeutralModeValue value) {
    return io.setNeutralMode(value);
  }

  // Pivot Getters

  public double getPivotVoltage() {
    return inputs.pivotAppliedVolts;
  }

  public Rotation2d getPivotPosition() {
    return inputs.pivotPosition;
  }

  public double getPivotVelocity() {
    return inputs.pivotVelocityDegreesPerSec;
  }

  public boolean isPivotIsAtTarget() {
    return isPivotIsAtTarget(targetPivotPosition);
  }

  public boolean isPivotIsAtTarget(Rotation2d target) {
    return isPivotIsAtTarget(target, Rotation2d.fromDegrees(pivotToleranceDegrees.get()));
  }

  public boolean isPivotIsAtTarget(Rotation2d target, Rotation2d tolerance) {
    return Math.abs(inputs.pivotPosition.getRadians() - target.getRadians())
        <= tolerance.getRadians();
  }
}
