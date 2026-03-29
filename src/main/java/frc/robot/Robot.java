// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.RaiderLog.RaiderLog;
import frc.lib.util.RaiderLog.RaiderLog.Importance;
import frc.lib.util.RaiderLog.RaiderLog.LogMode;
import frc.robot.Constants.RobotConstants;
import frc.robot.state.DriverDashboard;
import frc.robot.state.RobotStates;
import org.littletonrobotics.junction.LoggedRobot;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
	private Command m_autonomousCommand;

	private final RobotContainer m_robotContainer;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	public Robot() {

		super(RobotConstants.loopTimeSecs);

		// Set up logging
		Importance minImportance = Importance.DEBUG;
		LogMode logMode = LogMode.BASIC;
		RaiderLog.init(minImportance, logMode);
		if (logMode == LogMode.REPLAY) {
			setUseTiming(false); // Allows simulation to run as fast as possible
		}

		m_robotContainer = new RobotContainer();

		// Configure PhotonVision debug tab
	PortForwarder.add(5800, "photonvision.local", 5800);
		// CameraServer.startAutomaticCapture();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		RaiderLog.logAll();
		DriverDashboard.update();
		if (!RobotStates.isAllianceConfirmed && DriverStation.getAlliance().isPresent()) {
			RobotStates.isAllianceConfirmed = true;
			RobotStates.robotAlliance = DriverStation.getAlliance().get();
		}
	}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(m_autonomousCommand);
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			CommandScheduler.getInstance().cancel(m_autonomousCommand);
		}

		RobotContainer.resetSubsystems();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void simulationInit() {}

	@Override
	public void simulationPeriodic() {}
}
