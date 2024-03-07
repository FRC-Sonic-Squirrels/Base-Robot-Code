// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.commands.RunsWhenDisabledInstantCommand;
import frc.robot.autonomous.AutosManager.Auto;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private RobotContainer robotContainer;

  private LoggedDashboardChooser<String> autonomousChooser = null;
  private Auto selectedAuto;
  private Command autoCommand;
  private Pose2d selectedInitialPose;
  private Pose2d desiredInitialPose;
  private boolean hasEnteredTeleAtSomePoint = false;
  private boolean hasEnteredAutoAtSomePoint = false;

  // Enables power distribution logging
  private PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("OS", System.getProperty("os.name"));
    Logger.recordMetadata("Architecture", System.getProperty("os.arch"));

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.RobotMode.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        // FIXME: add a git ignored folder where logs are saved
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    var commandScheduler = CommandScheduler.getInstance();
    commandScheduler.onCommandInitialize(
        command -> Logger.recordOutput("ActiveCommands/" + command.getName(), true));
    commandScheduler.onCommandInterrupt(
        command -> Logger.recordOutput("ActiveCommands/" + command.getName(), false));
    commandScheduler.onCommandFinish(
        command -> Logger.recordOutput("ActiveCommands/" + command.getName(), false));
    robotContainer.resetSubsystems();

    // Dashboard buttons to turn off/on cameras
    SmartDashboard.putData(
        "PV Camera power OFF",
        new RunsWhenDisabledInstantCommand(() -> powerDistribution.setSwitchableChannel(false)));
    SmartDashboard.putData(
        "PV Camera power ON",
        new RunsWhenDisabledInstantCommand(() -> powerDistribution.setSwitchableChannel(true)));
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // Logger.recordOutput("Vision/", null);

    // FIXME: remove this eventually
    Logger.recordOutput("TIME/CTRE TIME", Utils.getCurrentTimeSeconds());
    Logger.recordOutput("TIME/FPGA TIME", Timer.getFPGATimestamp());
    Logger.recordOutput("TIME/REAL FPGA", Logger.getRealTimestamp());

    robotContainer.applyToDrivetrain();
    robotContainer.updateVisualization();

    Logger.recordOutput("DIO/0", robotContainer.breakModeButton.get());
    Logger.recordOutput("DIO/1", robotContainer.homeSensorsButton.get());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    robotContainer.enterDisabled();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    if (autonomousChooser == null) {
      autonomousChooser = robotContainer.getAutonomousChooser();
    }

    var currentChooserSelectedName = autonomousChooser.get();
    if (currentChooserSelectedName != null) {
      Pose2d initialPose;

      selectedAuto = robotContainer.getAutoSupplierForString(currentChooserSelectedName).get();
      if (selectedAuto != null) {
        initialPose = selectedAuto.initPose();
        if (initialPose != null) {
          initialPose = AllianceFlipUtil.flipPoseForAlliance(initialPose);
        }

        Logger.recordOutput("Auto/currentChooserValue", currentChooserSelectedName);
        Logger.recordOutput("Auto/SelectedAuto", selectedAuto.name());
      } else {
        initialPose = null;
      }

      desiredInitialPose = initialPose;
    }

    if (desiredInitialPose != null && !desiredInitialPose.equals(selectedInitialPose)) {
      boolean shouldResetPose = false;
      if (DriverStation.isFMSAttached()) {
        if (!hasEnteredTeleAtSomePoint) {
          shouldResetPose = true;
        }
      } else {
        shouldResetPose = true;
      }

      if (shouldResetPose) {
        selectedInitialPose = desiredInitialPose;
        robotContainer.setPose(selectedInitialPose);
      }
    }

    // set gyro zero based on vision during pre match disable. This allows for imprecise robot
    // placement on the field to be fixed by vision
    // fms = has ever entered auto or tele then never reset
    // no fms = always reset
    if (DriverStation.isFMSAttached()) {
      if (!hasEnteredAutoAtSomePoint && !hasEnteredTeleAtSomePoint) {
        robotContainer.matchRawOdometryToPoseEstimatorValue();
      }
    } else {
      robotContainer.matchRawOdometryToPoseEstimatorValue();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    robotContainer.enterAutonomous();

    // schedule the autonomous command (example)
    if (selectedAuto != null) {
      autoCommand = selectedAuto.command();
      autoCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    robotContainer.setBrakeMode();
    robotContainer.vision.useMaxDistanceAwayFromExistingEstimate(true);
    robotContainer.vision.useGyroBasedFilteringForVision(true);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autoCommand != null) {
      autoCommand.cancel();
      autoCommand = null;
    }

    robotContainer.enterTeleop();

    hasEnteredTeleAtSomePoint = true;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(robotContainer.getCurrentDraws()));

    Logger.recordOutput("SimulatedRobot/currentDraws", robotContainer.getCurrentDraws());

    Logger.recordOutput("SimulatedRobot/batteryVoltage", RobotController.getBatteryVoltage());
  }
}
