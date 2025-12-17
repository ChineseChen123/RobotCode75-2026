// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.TunerConstants.*;
import static frc.robot.Constants.VisionConstants.moduleMatrix;
import static frc.robot.Constants.VisionConstants.visionMatrix;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.RaiderLog.RaiderLog;
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.Drivetrain.ResetHeading;
import frc.robot.commands.Drivetrain.TeleopSwerve;
import frc.robot.commands.Drivetrain.XStance;
import frc.robot.state.Driver;
import frc.robot.state.Operator;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Drivetrain.controllers.ChezyController;
import frc.robot.subsystems.Drivetrain.controllers.RotationController;
import frc.robot.subsystems.Vision.AprilTagCamera;
import frc.robot.subsystems.Vision.Limelight;

public class RobotContainer {

	// private static final AprilTagCamera m_LeftFacingCamera =
	// 		new AprilTagCamera("Center_Cam", LeftFacingCameraPose);

	// private static final AprilTagCamera m_RightFacingCamera =
	// 		new AprilTagCamera("Coral_Cam", RightFacingCameraPose);

	// private static final AprilTagCamera m_HPCamera = new AprilTagCamera("HP_Cam", HPCameraPose);

	private static final Limelight m_Limelight = new Limelight();

	private static final Swerve m_Swerve =
			new Swerve(
					TunerConstants.DrivetrainConstants,
					0,
					moduleMatrix,
					visionMatrix,
					FrontLeft,
					FrontRight,
					BackLeft,
					BackRight);

	private static final ChezyController m_ChezyController = new ChezyController();

	private static final RotationController m_RotationController = new RotationController();

	// define OI controls
	private static final Driver m_Driver =
			new Driver(new Joystick(leftStickPort), new Joystick(rightStickPort));
	private static final Operator m_Operator =
			new Operator(new CommandXboxController(controllerPort));

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);
		configureLogging();
		configureDefaultCommands();
		configureJoystickBinds();
		configureControllerBinds();
		configureChooser();
	}

	private void configureLogging() {
		RaiderLog.register("Swerve", m_Swerve);
	}

	private void configureDefaultCommands() {
		m_Swerve.setDefaultCommand(
				new TeleopSwerve(
						m_Swerve,
						() -> m_Driver.leftY().getAsDouble(),
						() -> m_Driver.leftX().getAsDouble(),
						() -> m_Driver.rightX().getAsDouble()));
	}

	private void configureJoystickBinds() {
		m_Driver.getLeftButton(resetHeadingButton).onTrue(new ResetHeading(m_Swerve));
		m_Driver.getRightButton(xstanceButton).whileTrue(new XStance(m_Swerve));

		m_Driver
				.getRightButton(robotRelativeButton)
				.onTrue(new InstantCommand(() -> m_Swerve.toggleRobotRelative()))
				.onFalse(new InstantCommand(() -> m_Swerve.toggleFieldRelative()));

		// m_Driver
		// 		.getLeftButton(resetBranchCamButton)
		// 		.onTrue(new InstantCommand(() -> m_BranchCamera.reloadPipeline()).ignoringDisable(true));
	}

	public void configureControllerBinds() {}

	private void configureChooser() {}

	public Command getAutonomousCommand() {
		return null;
	}

	public static Swerve getSwerve() {
		return m_Swerve;
	}

	public static ChezyController getChezyController() {
		return m_ChezyController;
	}

	public static AprilTagCamera[] getAprilTagCameras() {
		// return new AprilTagCamera[] {m_LeftFacingCamera, m_RightFacingCamera, m_HPCamera};
		return new AprilTagCamera[] {};
	}

	public static Limelight getLimelight() {
		return m_Limelight;
	}

	public static Driver getDriver() {
		return m_Driver;
	}

	public static Operator getOperator() {
		return m_Operator;
	}
}
