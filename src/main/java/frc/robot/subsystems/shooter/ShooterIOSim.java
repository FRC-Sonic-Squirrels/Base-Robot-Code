package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.team2930.TalonFXArmSim;
import frc.lib.team2930.TalonFXSim;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {

  // Pivot objects

  private final TalonFXArmSim pivot =
      new TalonFXArmSim(
          new SingleJointedArmSim(
              DCMotor.getFalcon500Foc(1),
              ShooterConstants.Pivot.GEARING,
              SingleJointedArmSim.estimateMOI(Units.Feet.of(1.5).in(Units.Meters), 20.0),
              ShooterConstants.SHOOTER_LENGTH.in(Units.Meters),
              ShooterConstants.Pivot.MIN_ANGLE_RAD.getRadians(),
              ShooterConstants.Pivot.MAX_ANGLE_RAD.getRadians(),
              false,
              ShooterConstants.Pivot.SIM_INITIAL_ANGLE));

  private VoltageOut pivotOpenLoopControl = new VoltageOut(0);
  private MotionMagicVoltage pivotClosedLoopControl = new MotionMagicVoltage(0);

  // Launcher objects

  private final TalonFXSim launcher =
      new TalonFXSim(
          DCMotor.getFalcon500Foc(2),
          ShooterConstants.Launcher.GEARING,
          ShooterConstants.Launcher.MOI);

  private VoltageOut launcherOpenLoopControl = new VoltageOut(0);
  private VelocityVoltage launcherClosedLoopControl = new VelocityVoltage(0);

  public ShooterIOSim() {}

  @Override
  public void updateInputs(Inputs inputs) {

    // Update simulations
    pivot.update(Constants.kDefaultPeriod);
    launcher.update(Constants.kDefaultPeriod);

    // Pivot inputs

    inputs.pivotAppliedVolts = pivot.getVoltage();
    inputs.pivotVelocityRadsPerSec = pivot.getVelocity().in(Units.RadiansPerSecond);
    inputs.pivotPosition = new Rotation2d(pivot.getPosition());

    // Launcher inputs

    inputs.launcherAppliedVolts = launcher.getVoltage();
    inputs.launcherRPM = launcher.getVelocity().in(Units.RPM);
  }

  // Pivot Methods

  @Override
  public void setPivotVoltage(double volts) {
    pivot.setControl(pivotOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setPivotPosition(Rotation2d rot) {
    pivot.setControl(pivotClosedLoopControl.withPosition(rot.getRotations()));
  }

  @Override
  public void setPivotClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    config.Slot0 = slot0Configs;
    config.MotionMagic = mmConfigs;

    pivot.setConfig(config);
  }

  // Launcher Methods

  @Override
  public void setLauncherVoltage(double volts) {
    launcher.setControl(launcherOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setLauncherRPM(double rollerRPM) {
    launcher.setControl(launcherClosedLoopControl.withVelocity(rollerRPM));
  }

  @Override
  public void setLauncherClosedLoopConstants(
      double kP, double kV, double kS, double maxProfiledAcceleration) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kV = kV;
    slot0Configs.kS = kS;

    config.Slot0 = slot0Configs;

    launcher.setConfig(config);
  }
}
