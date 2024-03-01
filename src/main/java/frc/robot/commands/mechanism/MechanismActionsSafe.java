package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.commands.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class MechanismActionsSafe {

  public static LoggedTunableNumber climbDownElevatorVelocity =
      new LoggedTunableNumber("MechanismActionsSafe/climbDownElevatorVelocity", 500);
  public static LoggedTunableNumber climbDownElevatorAcceleration =
      new LoggedTunableNumber("MechanismActionsSafe/climbDownElevatorAcceleration", 500);

  public static Command loadingPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::loadingPosition);
  }

  public static Command ampFast(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::AmpFastPosition);
  }

  public static Command ampPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::ampPosition);
  }

  public static Command ampPositionToLoadPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::loadingPosition);
  }

  public static Command ampStage1Position(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::ampPosition);
  }

  public static Command ampStage2Position(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::ampStage2Position);
  }

  public static Command ampStage3Position(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::ampStage3Position);
  }

  public static Command climbPrepUnderStagePosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::climbPrepUnderStagePosition);
  }

  public static Command climbPrepPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::climbPrepPosition);
  }

  public static Command climbDownPosition(Elevator elevator, Arm arm) {
    return goToPositionParallelSetMotionConstraints(
        elevator,
        arm,
        MechanismPositions::climbDownPosition,
        () -> climbDownElevatorVelocity.get(),
        () -> climbDownElevatorAcceleration.get());
  }

  public static Command climbTrapPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::climbTrapPosition);
  }

  public static Command climbChainCheck(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::climbChainCheck);
  }

  public static Command climbFinalRestPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::climbFinalRestPosition);
  }

  public static Command deployReactionArms(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::deployReactionArmsStep1)
        .andThen(goToPositionParallel(elevator, arm, MechanismPositions::deployReactionArmsStep2));
  }

  private static Command goToPositionParallel(
      Elevator elevator, Arm arm, Supplier<MechanismPosition> position) {

    return new Command() {
      private double safeElevatorHeightInchesForIntake;
      private double maxLegalHeightInches;

      private double armLength;
      private double safeAngleIntakeDegrees;

      private Measure<Distance> targetElevatorHeight;
      private double targetElevatorHeightInches;
      private double elevatorToleranceInches = 0.2;

      private Rotation2d targetArmAngle;
      private double targetArmAngleDegrees;
      private final Rotation2d armTolerance = Rotation2d.fromDegrees(5.0);

      @Override
      public void initialize() {
        var targetPosition = position.get();
        targetElevatorHeight = targetPosition.elevatorHeight();
        targetElevatorHeightInches = targetElevatorHeight.in(Units.Inches);
        targetArmAngle = targetPosition.armAngle();
        targetArmAngleDegrees = targetArmAngle.getDegrees();

        safeElevatorHeightInchesForIntake =
            Constants.ElevatorConstants.SAFE_HEIGHT.in(Units.Inches);
        maxLegalHeightInches = Constants.ElevatorConstants.MAX_LEGAL_HEIGHT.in(Units.Inches);
        armLength = Constants.ArmConstants.ARM_LENGTH.in(Units.Inches);

        // Above this angle, we hit the intake
        safeAngleIntakeDegrees = Constants.ArmConstants.ARM_SAFE_ANGLE.getDegrees();
      }

      @Override
      public void execute() {
        var currentElevatorHeightInches = elevator.getHeightInches();
        var armAngle = arm.getAngle();
        var currentArmVerticalOffset = armLength * armAngle.getSin();
        var currentArmAngleDegrees = armAngle.getDegrees();

        boolean armCanRotateFreelyAtThisHeight =
            currentElevatorHeightInches + armLength < maxLegalHeightInches;

        Logger.recordOutput(
            "MechanismActionsSafe/armCanRotateFreelyAtThisHeight", armCanRotateFreelyAtThisHeight);

        //
        // Compute the maximum vertical extension of the arm during rotation.
        //
        double maxArmExtensionDuringRotation;
        var currentArmIsBehindElevator = currentArmAngleDegrees > 90;
        var targetArmIsBehindElevator = targetArmAngleDegrees > 90;

        if (currentArmIsBehindElevator != targetArmIsBehindElevator) {
          // It needs to go through the vertical position, assume max length
          maxArmExtensionDuringRotation = armLength;
        } else {
          // Arm staying on the same side of the elevator,
          // get the highest between current and target position.
          var targetArmVerticalOffset = armLength * targetArmAngle.getSin();
          maxArmExtensionDuringRotation =
              Math.max(targetArmVerticalOffset, currentArmVerticalOffset);
        }

        Logger.recordOutput(
            "MechanismActionsSafe/maxArmExtensionDuringRotation", maxArmExtensionDuringRotation);

        var maxElevatorHeadroom = maxLegalHeightInches - maxArmExtensionDuringRotation;
        var currentSafeHeight =
            Math.min(maxElevatorHeadroom - elevatorToleranceInches, targetElevatorHeightInches);

        String actionName;

        if (targetArmAngleDegrees < safeAngleIntakeDegrees) {
          //
          // Arm moving to the home position, more or less.
          //
          if (currentArmAngleDegrees < safeAngleIntakeDegrees) {
            //
            // Safe to move to target position directly
            //
            Logger.recordOutput("MechanismActionsSafe/state", "HomeDirectly");
            elevator.setHeight(targetElevatorHeight);
            arm.setAngle(targetArmAngle);
            return;
          }

          //
          // Move elevator at most to the safe height.
          //
          currentSafeHeight = Math.max(safeElevatorHeightInchesForIntake, currentSafeHeight);

          actionName = "Home";
        } else {
          actionName = "Target";
        }

        Logger.recordOutput("MechanismActionsSafe/currentSafeHeight", currentSafeHeight);

        boolean aboveMinimumHeight =
            currentElevatorHeightInches > currentSafeHeight - elevatorToleranceInches;

        boolean belowMaximumHeight =
            currentElevatorHeightInches + maxArmExtensionDuringRotation < maxLegalHeightInches;

        // We are not going to exceed the maximum legal height, start rotation.
        if (aboveMinimumHeight && belowMaximumHeight) {
          arm.setAngle(targetArmAngle);
          Logger.recordOutput("MechanismActionsSafe/state", actionName + "Arm");
        } else {
          Logger.recordOutput("MechanismActionsSafe/state", actionName + "NoArm");
        }

        elevator.setHeight(Units.Inches.of(currentSafeHeight));
      }

      @Override
      public boolean isFinished() {
        return elevator.isAtTarget(targetElevatorHeight)
            && arm.isAtTargetAngle(targetArmAngle, armTolerance);
      }
    };
  }

  private static Command goToPositionParallelSetMotionConstraints(
      Elevator elevator,
      Arm arm,
      Supplier<MechanismPosition> position,
      DoubleSupplier maxElevatorVelocity,
      DoubleSupplier maxElevatorAcceleration) {
    return Commands.sequence(
        setElevatorMotionMagicCommand(elevator, maxElevatorVelocity, maxElevatorAcceleration),
        goToPositionParallel(elevator, arm, position),
        setElevatorMotionMagicCommand(
            elevator,
            () -> elevator.getDefaultMotionMagicConstraints().maxVelocity,
            () -> elevator.getDefaultMotionMagicConstraints().maxAcceleration));
  }

  private static Command setElevatorMotionMagicCommand(
      Elevator elevator,
      DoubleSupplier maxElevatorVelocity,
      DoubleSupplier maxElevatorAcceleration) {
    return Commands.runOnce(
        () -> {
          double vel = maxElevatorVelocity.getAsDouble();
          double acc = maxElevatorAcceleration.getAsDouble();

          if (vel != elevator.getCurrentMotionMagicConstraints().maxVelocity
              || acc != elevator.getCurrentMotionMagicConstraints().maxAcceleration) {

            elevator.setMotionMagicConstraints(
                new Constraints(
                    maxElevatorVelocity.getAsDouble(), maxElevatorAcceleration.getAsDouble()));
          }
        },
        elevator);
  }
}
