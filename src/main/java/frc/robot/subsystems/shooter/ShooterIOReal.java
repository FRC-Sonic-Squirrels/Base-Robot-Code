package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants.KrakenConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.LauncherConstants;
import frc.robot.Constants.ShooterConstants.PivotConstants;

public class ShooterIOReal implements ShooterIO {

  private final TalonFX launcher = new TalonFX(Constants.CanIDs.SHOOTER_CAN_ID);
  private final TalonFX pivot = new TalonFX(Constants.CanIDs.SHOOTER_PIVOT_CAN_ID);

  private final StatusSignal<Double> pivotPosition;
  private final StatusSignal<Double> pivotVelocity;
  private final StatusSignal<Double> pivotVoltage;
  private final StatusSignal<Double> pivotCurrentAmps;
  private final StatusSignal<Double> pivotTempCelsius;

  private final StatusSignal<Double> launcherVelocity;
  private final StatusSignal<Double> launcherVoltage;
  private final StatusSignal<Double> launcherCurrentAmps;
  private final StatusSignal<Double> launcherTempCelsius;

  private final BaseStatusSignal[] refreshSet;

  // FIX: add FOC
  private final MotionMagicVoltage pivotClosedLoopControl =
      new MotionMagicVoltage(0).withEnableFOC(true);
  private final VoltageOut pivotOpenLoop = new VoltageOut(0.0).withEnableFOC(true);

  private final MotionMagicVelocityVoltage launcherClosedLoop =
      new MotionMagicVelocityVoltage(0.0).withEnableFOC(true);
  private final VoltageOut launcherOpenLoop = new VoltageOut(0.0).withEnableFOC(true);

  public ShooterIOReal() {
    // Launcher config
    TalonFXConfiguration launcherConfig = new TalonFXConfiguration();

    launcherConfig.CurrentLimits.SupplyCurrentLimit = LauncherConstants.SUPPLY_CURRENT_LIMIT;
    launcherConfig.CurrentLimits.SupplyCurrentThreshold =
        LauncherConstants.SUPPLY_CURRENT_THRESHOLD;
    launcherConfig.CurrentLimits.SupplyTimeThreshold = LauncherConstants.SUPPLY_TIME_THRESHOLD;
    launcherConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    launcherConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    launcherConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    launcherConfig.Feedback.SensorToMechanismRatio = ShooterConstants.LauncherConstants.GEARING;

    launcherConfig.Voltage.PeakReverseVoltage = 0.0;
    launcherConfig.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    launcher.getConfigurator().apply(launcherConfig);

    // Pivot config
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    pivotConfig.CurrentLimits.SupplyCurrentLimit = PivotConstants.SUPPLY_CURRENT_LIMIT;
    pivotConfig.CurrentLimits.SupplyCurrentThreshold = PivotConstants.SUPPLY_CURRENT_THRESHOLD;
    pivotConfig.CurrentLimits.SupplyTimeThreshold = PivotConstants.SUPPLY_TIME_THRESHOLD;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    pivotConfig.Feedback.SensorToMechanismRatio = PivotConstants.GEARING;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        PivotConstants.MAX_ANGLE_RAD.getRotations();
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        PivotConstants.MIN_ANGLE_RAD.getRotations();

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivot.getConfigurator().apply(pivotConfig);

    // Pivot status signals

    pivotPosition = pivot.getPosition();
    pivotVelocity = pivot.getVelocity();
    pivotVoltage = pivot.getMotorVoltage();
    pivotCurrentAmps = pivot.getStatorCurrent();
    pivotTempCelsius = pivot.getDeviceTemp();

    // Launcher status signals

    launcherVelocity = launcher.getVelocity();
    launcherVoltage = launcher.getMotorVoltage();
    launcherCurrentAmps = launcher.getStatorCurrent();
    launcherTempCelsius = launcher.getDeviceTemp();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(100, pivotPosition, launcherVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50, pivotVelocity, pivotVoltage, pivotCurrentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(10, launcherVoltage, launcherCurrentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, launcherTempCelsius, pivotTempCelsius);

    launcher.optimizeBusUtilization();
    pivot.optimizeBusUtilization();

    refreshSet =
        new BaseStatusSignal[] {
          pivotPosition,
          pivotVelocity,
          pivotVoltage,
          pivotCurrentAmps,
          pivotTempCelsius,
          // --
          launcherVelocity,
          launcherVoltage,
          launcherCurrentAmps,
          launcherTempCelsius
        };
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.refreshAll(refreshSet);

    inputs.pivotPosition = Rotation2d.fromRotations(pivotPosition.getValueAsDouble());
    inputs.pivotVelocityDegreesPerSec =
        Units.RotationsPerSecond.of(pivotVelocity.getValueAsDouble()).in(Units.DegreesPerSecond);
    inputs.pivotAppliedVolts = pivotVoltage.getValueAsDouble();
    inputs.pivotCurrentAmps = pivotCurrentAmps.getValueAsDouble();

    inputs.launcherRPM =
        Units.RotationsPerSecond.of(launcherVelocity.getValueAsDouble()).in(Units.RPM);

    inputs.launcherAppliedVolts = launcherVoltage.getValueAsDouble();

    inputs.launcherCurrentAmps = launcherCurrentAmps.getValueAsDouble();

    inputs.tempsCelcius[0] = launcherTempCelsius.getValueAsDouble();
    inputs.tempsCelcius[1] = pivotTempCelsius.getValueAsDouble();
  }

  // PIVOT

  @Override
  public void setPivotPosition(Rotation2d rot) {
    pivotClosedLoopControl.withPosition(rot.getRotations());
    pivot.setControl(pivotClosedLoopControl);
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivot.setControl(pivotOpenLoop.withOutput(volts));
  }

  // LAUNCHER
  @Override
  public void setLauncherVoltage(double volts) {
    launcher.setControl(launcherOpenLoop.withOutput(volts));
  }

  @Override
  public void setLauncherRPM(double rollerRPM) {
    launcher.setControl(launcherClosedLoop.withVelocity(rollerRPM / 60));
  }

  @Override
  public void resetPivotSensorPosition(Rotation2d position) {
    pivot.setPosition(position.getRotations());
  }

  @Override
  public void setPivotClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {
    Slot0Configs pidConfig = new Slot0Configs();

    var configurator = pivot.getConfigurator();
    configurator.refresh(pidConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;

    configurator.apply(pidConfig);
    configurator.apply(mmConfigs);
  }

  @Override
  public void setLauncherClosedLoopConstants(
      double kP, double kV, double kS, double targetAccelerationConfig) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    var leadConfigurator = launcher.getConfigurator();

    leadConfigurator.refresh(pidConfig);
    leadConfigurator.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kV = kV;
    pidConfig.kS = kS;

    mmConfig.MotionMagicAcceleration = targetAccelerationConfig;

    leadConfigurator.apply(pidConfig);
    leadConfigurator.apply(mmConfig);
  }

  @Override
  public boolean setNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = pivot.getConfigurator().refresh(config);

    if (status != StatusCode.OK) return false;

    config.NeutralMode = value;

    pivot.getConfigurator().apply(config);
    return true;
  }
}
