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

public class Shooter extends SubsystemBase {
  private static final String ROOT_TABLE = "Shooter";
  private static final double MAX_VOLTAGE = 12;

  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  // Logging
  private static final LoggerGroup logInputs = LoggerGroup.build(ROOT_TABLE);

  // Pivot Logging
  private static final LoggerEntry.Decimal logInputs_pivotPosition =
      logInputs.buildDecimal("Pivot/Position");
  private static final LoggerEntry.Decimal logInputs_pivotVelocityRadsPerSec =
      logInputs.buildDecimal("Pivot/VelocityRadsPerSec");
  private static final LoggerEntry.Decimal logInputs_pivotAppliedVolts =
      logInputs.buildDecimal("Pivot/AppliedVolts");
  private static final LoggerEntry.Decimal logInputs_pivotCurrentAmps =
      logInputs.buildDecimal("Pivot/CurrentAmps");

  private static final LoggerEntry.Decimal logPivotTargetAngleDegrees =
      logInputs.buildDecimal("Pivot/TargetAngleDegrees");

  // Launcher Logging
  private static final LoggerEntry.DecimalArray logInputs_launcherRPM =
      logInputs.buildDecimalArray("Launcher/RPM");
  private static final LoggerEntry.DecimalArray logInputs_launcherAppliedVolts =
      logInputs.buildDecimalArray("Launcher/AppliedVolts");
  private static final LoggerEntry.DecimalArray logInputs_launcherCurrentAmps =
      logInputs.buildDecimalArray("Launcher/CurrentAmps");
  private static final LoggerEntry.DecimalArray logInputs_tempsCelcius =
      logInputs.buildDecimalArray("Launcher/TempsCelcius");

  private static final LoggerEntry.DecimalArray logLauncherTargetRPM =
      logInputs.buildDecimalArray("Launcher/TargetRPM");

  // Tunable Numbers
  public static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);

  // Pivot Tunable Numbers
  private static final TunableNumberGroup groupPivot = group.subgroup("Pivot");
  private static final LoggedTunableNumber pivotkP = groupPivot.build("kP");
  private static final LoggedTunableNumber pivotkD = groupPivot.build("kD");
  private static final LoggedTunableNumber pivotkG = groupPivot.build("kG");
  private static final LoggedTunableNumber pivotClosedLoopMaxVelocityConstraint =
      groupPivot.build("ClosedLoopMaxVelocityConstraint");
  private static final LoggedTunableNumber pivotClosedLoopMaxAccelerationConstraint =
      groupPivot.build("ClosedLoopMaxAccelerationConstraint");
  private static final LoggedTunableNumber pivotToleranceDegrees =
      groupPivot.build("pivotToleranceDegrees", 0.5);

  // Launcher Tunable Numbers
  private static final TunableNumberGroup groupLauncher = group.subgroup("Launcher");
  private static final LoggedTunableNumber launcherkS = groupLauncher.build("kS");
  private static final LoggedTunableNumber launcherkP = groupLauncher.build("kP");
  private static final LoggedTunableNumber launcherkV = groupLauncher.build("kV");
  private static final LoggedTunableNumber launcherClosedLoopMaxAccelerationConstraint =
      groupLauncher.build("ClosedLoopMaxAccelerationConstraint");
  private static final LoggedTunableNumber launcherToleranceRPM =
      groupLauncher.build("launcherToleranceRPM", 150); // TODO: tune for better tolerance

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_COMPETITION) {

      pivotkP.initDefault(800.0);
      pivotkD.initDefault(0.0);
      pivotkG.initDefault(0.0);
      pivotClosedLoopMaxVelocityConstraint.initDefault(10.0);
      pivotClosedLoopMaxAccelerationConstraint.initDefault(5.0);

      launcherkS.initDefault(0.24);
      launcherkP.initDefault(0.4);
      launcherkV.initDefault(0.072);
      launcherClosedLoopMaxAccelerationConstraint.initDefault(1000.0);

    } else if (Constants.RobotMode.isSimBot()) {

      pivotkP.initDefault(15.0);
      pivotkD.initDefault(0.0);
      pivotkG.initDefault(0.0);
      pivotClosedLoopMaxVelocityConstraint.initDefault(10.0);
      pivotClosedLoopMaxAccelerationConstraint.initDefault(10.0);

      launcherkS.initDefault(0.0);
      launcherkP.initDefault(0.05);
      launcherkV.initDefault(1);
      launcherClosedLoopMaxAccelerationConstraint.initDefault(0.0);
    }
  }

  private final ShooterIO io;
  private final ShooterIO.Inputs inputs = new ShooterIO.Inputs(logInputs);

  private Rotation2d targetPivotPosition = Constants.zeroRotation2d;
  private double rollerTargetRPM = 0.0;

  /** Creates a new ShooterSubsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;

    setLauncherClosedLoopConstants();

    setPivotClosedLoopConstants();
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      io.updateInputs(inputs);

      // Pivot logging
      logInputs_pivotPosition.info(inputs.pivotPosition);
      logInputs_pivotVelocityRadsPerSec.info(inputs.pivotVelocityRadsPerSec);
      logInputs_pivotAppliedVolts.info(inputs.pivotAppliedVolts);
      logInputs_pivotCurrentAmps.info(inputs.pivotCurrentAmps);

      // Launcher logging
      logInputs_launcherRPM.info(inputs.launcherRPM);
      logInputs_launcherAppliedVolts.info(inputs.launcherAppliedVolts);
      logInputs_launcherCurrentAmps.info(inputs.launcherCurrentAmps);
      logInputs_tempsCelcius.info(inputs.tempsCelcius);

      // Update constants
      var hc = hashCode();
      if (launcherkS.hasChanged(hc)
          || launcherkP.hasChanged(hc)
          || launcherkV.hasChanged(hc)
          || launcherClosedLoopMaxAccelerationConstraint.hasChanged(hc)) {
        setLauncherClosedLoopConstants();
      }

      if (pivotkP.hasChanged(hc)
          || pivotkD.hasChanged(hc)
          || pivotkG.hasChanged(hc)
          || pivotClosedLoopMaxVelocityConstraint.hasChanged(hc)
          || pivotClosedLoopMaxAccelerationConstraint.hasChanged(hc)) {

        setPivotClosedLoopConstants();
      }
    }
  }

  // Launcher setters

  public void setLauncherPercentOut(double percent) {
    io.setLauncherVoltage(percent * MAX_VOLTAGE);
  }

  public void setLauncherRPM(double rpm) {
    io.setLauncherRPM(rpm);
    rollerTargetRPM = rpm;
    logLauncherTargetRPM.info(rpm);
  }

  public void setLauncherClosedLoopConstants() {
    io.setLauncherClosedLoopConstants(
        launcherkP.get(),
        launcherkV.get(),
        launcherkS.get(),
        launcherClosedLoopMaxAccelerationConstraint.get());
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
  }

  public void setPivotPosition(Rotation2d rot) {
    io.setPivotPosition(rot);
    targetPivotPosition = rot;
    logPivotTargetAngleDegrees.info(rot);
  }

  public void setPivotClosedLoopConstants() {
    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();

    mmConfigs.MotionMagicAcceleration = pivotClosedLoopMaxAccelerationConstraint.get();
    mmConfigs.MotionMagicCruiseVelocity = pivotClosedLoopMaxVelocityConstraint.get();

    io.setPivotClosedLoopConstants(pivotkP.get(), pivotkD.get(), pivotkG.get(), mmConfigs);
  }

  public void pivotResetHomePosition() {
    io.resetPivotSensorPosition(Constants.ShooterConstants.Pivot.HOME_POSITION);
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
    return inputs.pivotVelocityRadsPerSec;
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
