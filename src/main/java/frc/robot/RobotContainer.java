// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.IOConstants.*;
import static frc.robot.Constants.VisionConstants.HPCameraPose;
import static frc.robot.Constants.VisionConstants.LeftFacingCameraPose;
import static frc.robot.Constants.VisionConstants.RightFacingCameraPose;
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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.Vision.AprilTagCamera;
import frc.robot.subsystems.Vision.ObjectDetetectorCamera;

public class RobotContainer {

	// Initialize cameras
	private static final AprilTagCamera m_LeftFacingCamera =
			new AprilTagCamera("Center_Cam", LeftFacingCameraPose);

	private static final AprilTagCamera m_RightFacingCamera =
			new AprilTagCamera("Coral_Cam", RightFacingCameraPose);

	private static final AprilTagCamera m_HPCamera = new AprilTagCamera("HP_Cam", HPCameraPose);

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

	private static final Elevator m_Elevator = new Elevator();

	private static final CoralIntake m_CoralIntake = new CoralIntake();

	private static final Climber m_Climber = new Climber();

	private static final AlgaeIntake m_AlgaeIntake = new AlgaeIntake();

	private static final AlgaePivot m_AlgaePivot = new AlgaePivot();

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
		RaiderLog.register("Elevator", m_Elevator);
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

	public static Elevator getElevator() {
		return m_Elevator;
	}

	public static CoralIntake getCoralIntake() {
		return m_CoralIntake;
	}

	public static Climber getClimber() {
		return m_Climber;
	}

	public static AlgaeIntake getAlgaeIntake() {
		return m_AlgaeIntake;
	}

	public static AlgaePivot getAlgaePivot() {
		return m_AlgaePivot;
	}

	public static AprilTagCamera[] getAprilTagCameras() {
		return new AprilTagCamera[] {m_LeftFacingCamera, m_RightFacingCamera, m_HPCamera};
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
