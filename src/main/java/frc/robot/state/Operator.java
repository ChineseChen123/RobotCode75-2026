// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.state;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.DrivetrainConstants.maxAngularVelocity;
import static frc.robot.Constants.DrivetrainConstants.maxVelocity;
import static frc.robot.Constants.IOConstants.stickDeadband;
import static frc.robot.Constants.IOConstants.translationJoystickExpo;
import static frc.robot.Constants.IOConstants.translationStickMapValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

public class Operator extends SubsystemBase {

	public static final Trigger kFalse = new Trigger(() -> false);

	private final CommandXboxController m_Controller;

	public Trigger A = kFalse;
	public Trigger B = kFalse;
	public Trigger X = kFalse;
	public Trigger Y = kFalse;
	public Trigger leftBumper = kFalse;
	public Trigger rightBumper = kFalse;
	public Trigger upDpad = kFalse;
	public Trigger downDpad = kFalse;
	public Trigger leftDpad = kFalse;
	public Trigger rightDpad = kFalse;
	public Trigger start = kFalse;
	public Trigger back = kFalse;
	

	/** Creates a new Driver. */
	public Operator(CommandXboxController controller) {
		m_Controller = controller;

		A = m_Controller.a();
		B = m_Controller.b();
		X = m_Controller.x();
		Y = m_Controller.y();
		leftBumper = m_Controller.leftBumper();
		rightBumper = m_Controller.rightBumper();
		upDpad = m_Controller.povUp();
		downDpad = m_Controller.povDown();
		leftDpad = m_Controller.povLeft();
		rightDpad = m_Controller.povRight();
		start = m_Controller.start();
		back = m_Controller.back();
	}

	public Trigger leftTriggerGreater(double thresh) {
		return new Trigger(() -> m_Controller.getLeftTriggerAxis() > thresh);
	}

	public Trigger rightTriggerGreater(double thresh) {
		return new Trigger(() -> m_Controller.getRightTriggerAxis() > thresh);
	}

	public DoubleSupplier leftStickX() {
		return () -> m_Controller.getLeftX();
	}

	public DoubleSupplier leftStickY() {
		return () -> m_Controller.getLeftY();
	}

	/** applies deadbands and exponents to stick values */
	public DoubleSupplier leftStickXProcessed() {
		return () -> {
			double val =
					MathUtil.applyDeadband(m_Controller.getLeftX(), stickDeadband) * translationStickMapValue;
			return val >= 0
					? Math.pow(val, translationJoystickExpo)
					: -1 * Math.pow(-val, translationJoystickExpo);
		};
	}

	/** applies deadbands and exponents to stick values */
	public DoubleSupplier leftStickYProcessed() {
		return () -> {
			double val =
					MathUtil.applyDeadband(m_Controller.getLeftY(), stickDeadband) * translationStickMapValue;
			return val >= 0
					? Math.pow(val, translationJoystickExpo)
					: -1 * Math.pow(-val, translationJoystickExpo);
		};
	}

	public DoubleSupplier rightStickX() {
		return () -> m_Controller.getRightX();
	}

	public DoubleSupplier rightStickY() {
		return () -> m_Controller.getRightY();
	}

	/** applies deadbands and exponents to stick values */
	public DoubleSupplier rightStickXProcessed() {
		return () -> {
			double val =
					MathUtil.applyDeadband(m_Controller.getRightX(), stickDeadband)
							* translationStickMapValue;
			return val >= 0
					? Math.pow(val, translationJoystickExpo)
					: -1 * Math.pow(-val, translationJoystickExpo);
		};
	}

	public void rumble(double leftIntensity, double rightIntensity) {
		if (!m_Controller.getHID().isConnected()) {
			return;
		}
		m_Controller.getHID().setRumble(RumbleType.kLeftRumble, leftIntensity);
		m_Controller.getHID().setRumble(RumbleType.kRightRumble, rightIntensity);
	}

	/** returns array of all 3 processed joystick values (for one driver) */
	public double[] processedJoystickValues() {
		// Negation because joystick forward is negative
		double[] DriverInput = {
			MathUtil.applyDeadband(-leftStickYProcessed().getAsDouble(), stickDeadband),
			MathUtil.applyDeadband(-leftStickXProcessed().getAsDouble(), stickDeadband),
			MathUtil.applyDeadband(-rightStickXProcessed().getAsDouble(), stickDeadband)
		};
		boolean fieldRelative = RobotContainer.getSwerve().getFieldRelative();
		if (!fieldRelative) {
			DriverInput[0] *= 0.5;
			DriverInput[1] *= 0.5;
			DriverInput[2] *= 0.5;
		} else if (DriverStation.getAlliance().get() == Alliance.Red) {
			DriverInput[0] *= -1;
			DriverInput[1] *= -1;
		}
		DriverInput[0] *= maxVelocity.in(MetersPerSecond);
		DriverInput[1] *= maxVelocity.in(MetersPerSecond);
		DriverInput[2] *= maxAngularVelocity.in(RadiansPerSecond);

		return DriverInput;
	}
}
