package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.lib.team2930.LoggerGroup;
import frc.robot.subsystems.BaseInputs;

public interface ElevatorIO {
  /** Contains all of the input data received from hardware. */
  class Inputs extends BaseInputs {
    public double heightInches = 0.0;
    public double velocityInchesPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;

    public Inputs(LoggerGroup logInputs) {
      super(logInputs);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setHeight(Measure<Distance> height) {}

  public default void setPIDConstraints(double kP, double kD, double kG, Constraints constraints) {}

  public default void setSensorPosition(Measure<Distance> position) {}

  public default boolean setNeutralMode(NeutralModeValue value) {
    return false;
  }
}
