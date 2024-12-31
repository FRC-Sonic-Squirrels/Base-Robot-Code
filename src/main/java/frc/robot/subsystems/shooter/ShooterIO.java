package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BaseInputs;

public interface ShooterIO {
  /** Contains all of the input data received from hardware. */
  class Inputs extends BaseInputs {
    public Rotation2d pivotPosition = Constants.zeroRotation2d;
    public double pivotVelocityDegreesPerSec;
    public double pivotAppliedVolts;
    public double pivotCurrentAmps;

    public double launcherRPM;
    public double launcherAppliedVolts;
    public double launcherCurrentAmps;

    // launcher, pivot
    public double[] tempsCelcius = new double[2];

    public Inputs(LoggerGroup logInputs) {
      super(logInputs);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {}

  // Pivot
  public default void setPivotPosition(Rotation2d rot) {}

  public default void setPivotVoltage(double volts) {}

  public default void resetPivotSensorPosition(Rotation2d position) {}

  public default void setPivotClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {}

  // Launcher
  public default void setLauncherVoltage(double volts) {}

  public default void setLauncherRPM(double rollerRPM) {}

  public default void setLauncherClosedLoopConstants(
      double kP, double kV, double kS, double targetAccelerationConfig) {}

  public default boolean setNeutralMode(NeutralModeValue value) {
    return false;
  }
}
