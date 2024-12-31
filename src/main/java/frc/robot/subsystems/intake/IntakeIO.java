package frc.robot.subsystems.intake;

import frc.lib.team2930.LoggerGroup;
import frc.robot.subsystems.BaseInputs;

public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  class Inputs extends BaseInputs {
    public double velocityRPM;
    public double currentAmps;
    public double tempCelsius;
    public double appliedVolts;

    public Inputs(LoggerGroup logInputs) {
      super(logInputs);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double revPerMin) {}

  public default void setClosedLoopConstants(
      double kP, double kV, double kS, double targetAccelerationConfig) {}
}
