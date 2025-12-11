// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.IOConstants.*;
import static frc.robot.Constants.VisionConstants.moduleMatrix;
import static frc.robot.Constants.VisionConstants.visionMatrix;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.dashboard.AutoSelector;
import frc.lib.util.RaiderLog.RaiderLog;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.state.Bindings;
import frc.robot.state.Driver;
import frc.robot.state.Operator;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.AprilTagCamera;
import frc.robot.subsystems.Vision.ObjectDetetectorCamera;

public class RobotContainer {


	private static final ObjectDetetectorCamera m_BranchCamera =
			new ObjectDetetectorCamera("Branch_Cam");

	// Initialize Phoenix swerve
	private static final Swerve m_Swerve =
			new Swerve(
					DrivetrainConstants.SwerveDrivetrainConstants,
					0, // Defaults to 250 hz
					moduleMatrix,
					visionMatrix,
					DrivetrainConstants.FrontLeft,
					DrivetrainConstants.FrontRight,
					DrivetrainConstants.BackLeft,
					DrivetrainConstants.BackRight);


	// Define IO controls
	private static final Driver m_Driver =
			new Driver(new Joystick(leftStickPort), new Joystick(rightStickPort));
	private static final Operator m_Operator =
			new Operator(new CommandXboxController(controllerPort));
	private static final Bindings m_Bindings = new Bindings();

	private final AutoSelector m_AutoSelector = new AutoSelector();

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);
		configureLogging();
		configureBinds();
		configureChooser();
	}

	// Register any subsystems to be logged
	private void configureLogging() {
		RaiderLog.register("Swerve", m_Swerve);
		// RaiderLog.register("Elevator", m_Elevator);
		// RaiderLog.register("Coral Intake", m_CoralIntake);
		// RaiderLog.register("Algae Intake", m_AlgaeIntake);
		// RaiderLog.register("Algae Pivot", m_AlgaePivot);
		// RaiderLog.register("Climber", m_Climber);
	}

	// Configure button bindings based on driving mode
	public void configureBinds() {
		if (oneDriver) {
			m_Bindings.bind1Driver();
		} else {
			m_Bindings.bind2Driver();
		}
	}

	// Configure auto selector
	private void configureChooser() {
		m_AutoSelector.setupAutoTab();
		m_AutoSelector.clearAll();
	}

	public Command getAutonomousCommand() {
		m_AutoSelector.generatePaths();
		return m_AutoSelector.getAutoCommand();
	}

	// Methods to return instances of static subsystems

	public static Swerve getSwerve() {
		return m_Swerve;
	}

	public static ObjectDetetectorCamera getBranchCamera() {
		return m_BranchCamera;
	}

	public static Driver getDriver() {
		return m_Driver;
	}

	public static Operator getOperator() {
		return m_Operator;
	}
}
