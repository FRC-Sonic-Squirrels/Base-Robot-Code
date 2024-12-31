package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MotorConstants.KrakenConstants;

public class ArmIOReal implements ArmIO {
  private final StatusSignal<Double> appliedVols;
  private final StatusSignal<Double> positionRotations;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> tempCelsius;
  private final StatusSignal<Double> velocity;

  private final MotionMagicVoltage closedLoopControl =
      new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(true);

  private final TalonFX motor = new TalonFX(Constants.CanIDs.ARM_CAN_ID);
  ;

  private final BaseStatusSignal[] refreshSet;

  public ArmIOReal() {
    // Motor config
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = ArmConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.ArmConstants.MAX_ARM_ANGLE.getRotations();
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.ArmConstants.MIN_ARM_ANGLE.minus(Rotation2d.fromDegrees(2.0)).getRotations();

    config.Feedback.SensorToMechanismRatio = Constants.ArmConstants.GEAR_RATIO;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    motor.getConfigurator().apply(config);

    // Status signals

    appliedVols = motor.getMotorVoltage();
    positionRotations = motor.getPosition();
    currentAmps = motor.getStatorCurrent();
    tempCelsius = motor.getDeviceTemp();
    velocity = motor.getVelocity();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(100, appliedVols, positionRotations, velocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50, currentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, tempCelsius);

    motor.optimizeBusUtilization();

    refreshSet =
        new BaseStatusSignal[] {appliedVols, positionRotations, currentAmps, tempCelsius, velocity};
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.refreshAll(refreshSet);

    inputs.armPosition = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.armAppliedVolts = appliedVols.getValueAsDouble();
    inputs.armCurrentAmps = currentAmps.getValueAsDouble();
    inputs.armTempCelsius = tempCelsius.getValueAsDouble();
    inputs.armVelocityDegreesPerSecond =
        Units.RotationsPerSecond.of(velocity.getValueAsDouble()).in(Units.DegreesPerSecond);
  }

  @Override
  public void setClosedLoopPosition(Rotation2d angle) {
    closedLoopControl.withPosition(angle.getRotations());
    motor.setControl(closedLoopControl);
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {
    var slot0Configs = new Slot0Configs();

    motor.getConfigurator().refresh(slot0Configs);
    motor.getConfigurator().refresh(mmConfigs);

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    motor.getConfigurator().apply(slot0Configs);
    motor.getConfigurator().apply(mmConfigs);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(openLoopControl.withOutput(volts));
  }

  @Override
  public void resetSensorPosition(Rotation2d angle) {
    motor.setPosition(angle.getRotations());
  }

  @Override
  public boolean setNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = motor.getConfigurator().refresh(config);

    if (status != StatusCode.OK) return false;

    config.NeutralMode = value;

    motor.getConfigurator().apply(config);
    return true;
  }
}
