// Copyright 2021-2024 FRC 6328
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

import edu.wpi.first.wpilibj2.command.Command;
import frc.game.FieldConstants;
import frc.robot.power.RobotPowerDistribution;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import org.ciprix.CiprixRobot;
import org.ciprix.CiprixSettings;
import org.ciprix.util.drivetrain.PIDRotation;
import org.ciprix.util.security.OSUtil;
import org.littletonrobotics.junction.Logger;

/**
 * The robot: extends {@link CiprixRobot}, which configures AdvantageKit (USB + NetworkTables on the
 * robot, NetworkTables in sim, log replay), the {@link CiprixSettings}, the pre-match checklist and
 * the {@code CommandScheduler} lifecycle. The structure (subsystems, buttons, autonomous) lives in
 * {@link RobotContainer}.
 */
public class Robot extends CiprixRobot {
  private final RobotContainer robotContainer;

  public Robot() {
    super(Constants.simMode, Constants.tuningMode);
    RobotPowerDistribution.getInstance();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** Runs before AdvantageKit starts and before {@link RobotContainer} is created. */
  @Override
  protected void configure() {
    CiprixSettings.setControllerDeadband(Constants.CONTROLLER_DEADBAND);
    CiprixSettings.setDefaultAlliance(Constants.alliance);
    CiprixSettings.setFieldSize(FieldConstants.fieldLength, FieldConstants.fieldWidth);
    CiprixSettings.setPeriodicTimerEnabled(Constants.periodicTimer);
    PIDRotation.setDefaultGains(
        DrivetrainConstants.ANGLE_KP, DrivetrainConstants.ANGLE_KI, DrivetrainConstants.ANGLE_KD);

    Logger.recordMetadata("JavaVersion", System.getProperty("java.version"));
    Logger.recordMetadata("OS", System.getProperty("os.name"));
    Logger.recordMetadata("User", System.getProperty("user.name"));
  }

  @Override
  protected String getProjectName() {
    return "TechMaker_Swerve_2026";
  }

  @Override
  protected Command getAutonomousCommand() {
    return robotContainer.getAutonomousCommand();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    OSUtil.fsSyncAsync();
  }
}
