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

	private final Swerve swerve;

	private PIDController controller;

	public RotationController() {
		controller =
				new PIDController(
						kp, 0, // no I term
						kd);
		controller.enableContinuousInput(-Math.PI, Math.PI);

		controller.setTolerance(toleranceRadians);
		this.swerve = RobotContainer.getSwerve();
	}

	public double getOutput() {
		return output;
	}

	public void update(Rotation2d setpoint) {
		controller.setSetpoint(setpoint.getRadians());
		this.output =
				controller.calculate(swerve.getHeading().getRadians(), setpoint.getRadians()) + 0.03;
	}

	public void update(Rotation2d setpoint, double p, double d) {
		// Update controller
		controller.setPID(p, 0, d);
		update(setpoint);
	}

	public boolean atGoal() {
		return controller.atSetpoint() || this.output <= 0.03;
	}
}
