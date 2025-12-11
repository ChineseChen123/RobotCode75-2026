// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain.controllers;

import static frc.robot.Constants.DrivetrainConstants.ControllerConstants.RotationAlign.*;
import static frc.robot.Constants.DrivetrainConstants.ControllerConstants.toleranceRadians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Swerve;

public class RotationController {
	private double output;

	private Swerve m_swerve;

	private PIDController controller;

	public RotationController() {
		controller =
				new PIDController(
						kp, 0, // no I term
						kd);
		
		// makes -pi equal to pi to prevent rotating in the wrong direction
		controller.enableContinuousInput(-Math.PI, Math.PI);

		controller.setTolerance(toleranceRadians);
		this.m_swerve = RobotContainer.getSwerve();
	}

	public double getOutput() {
		return output;
	}

	/** updates PIDController with our current heading and target heading */
	public void update(Rotation2d setpoint) {
		if (m_swerve == null) {
			m_swerve = RobotContainer.getSwerve(); // initialize swerve if not done already
		}
		controller.setSetpoint(setpoint.getRadians());
		this.output =
				controller.calculate(m_swerve.getHeading().getRadians(), setpoint.getRadians()) + 0.03;
	}

	/** returns whether we are at our target heading */
	public boolean atGoal() {
		return controller.atSetpoint() || this.output <= 0.03;
	}
}
