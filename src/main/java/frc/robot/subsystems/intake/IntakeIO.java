package frc.robot.subsystems.intake;

import frc.lib.team2930.LoggerGroup;
import frc.robot.subsystems.BaseInputs;

public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  class Inputs extends BaseInputs {
    public double velocityRPM = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double appliedVolts = 0.0;

    public Inputs(LoggerGroup logInputs) {
      super(logInputs);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double revPerMin) {}

  public default void setClosedLoopConstants(
      double kP, double kV, double kS, double maxProfiledAcceleration) {}
}
