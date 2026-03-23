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

	public DoubleSupplier leftStickX = () -> 0;
	public DoubleSupplier leftStickY = () -> 0;
	public DoubleSupplier leftStickXProcessed = () -> 0;
	public DoubleSupplier leftStickYProcessed = () -> 0;
	public DoubleSupplier rightStickX = () -> 0;
	public DoubleSupplier rightStickY = () -> 0;
	public DoubleSupplier rightStickXProcessed = () -> 0;
	public DoubleSupplier rightStickYProcessed = () -> 0;

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

		leftStickX = () -> m_Controller.getHID().getLeftX();
		leftStickY = () -> m_Controller.getHID().getLeftY();
		leftStickXProcessed =
				() -> {
					double val =
							MathUtil.applyDeadband(m_Controller.getHID().getLeftX(), stickDeadband)
									* translationStickMapValue;
					return val >= 0
							? Math.pow(val, translationJoystickExpo)
							: -1 * Math.pow(-val, translationJoystickExpo);
				};
		leftStickYProcessed =
				() -> {
					double val =
							MathUtil.applyDeadband(m_Controller.getHID().getLeftY(), stickDeadband)
									* translationStickMapValue;
					return val >= 0
							? Math.pow(val, translationJoystickExpo)
							: -1 * Math.pow(-val, translationJoystickExpo);
				};
		rightStickX = () -> m_Controller.getHID().getRightX();
		rightStickY = () -> m_Controller.getHID().getRightY();
		rightStickXProcessed =
				() -> {
					double val =
							MathUtil.applyDeadband(m_Controller.getHID().getRightX(), stickDeadband)
									* translationStickMapValue;
					return val >= 0
							? Math.pow(val, translationJoystickExpo)
							: -1 * Math.pow(-val, translationJoystickExpo);
				};
		rightStickYProcessed =
				() -> {
					double val =
							MathUtil.applyDeadband(m_Controller.getHID().getRightY(), stickDeadband)
									* translationStickMapValue;
					return val >= 0
							? Math.pow(val, translationJoystickExpo)
							: -1 * Math.pow(-val, translationJoystickExpo);
				};
	}

	public Trigger leftTriggerGreater(double thresh) {
		return new Trigger(() -> m_Controller.getHID().getLeftTriggerAxis() > thresh);
	}

	public Trigger rightTriggerGreater(double thresh) {
		return new Trigger(() -> m_Controller.getHID().getRightTriggerAxis() > thresh);
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
			MathUtil.applyDeadband(-leftStickYProcessed.getAsDouble(), stickDeadband),
			MathUtil.applyDeadband(-leftStickXProcessed.getAsDouble(), stickDeadband),
			MathUtil.applyDeadband(-rightStickXProcessed.getAsDouble(), stickDeadband)
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

	double lastJoystickAngle = 0;

	public double[] processedJoystickValuesPositionalRotation() {
		// Negation because joystick forward is negative
		double[] DriverInput = processedJoystickValues();

		// get normalized vector of rotation translation

		double magnitude =
				Math.sqrt(
						Math.pow(rightStickXProcessed.getAsDouble(), 2)
								+ Math.pow(rightStickYProcessed.getAsDouble(), 2));
		if (magnitude > 0.1) {
			double angle =
					Math.atan2(rightStickYProcessed.getAsDouble(), rightStickXProcessed.getAsDouble());
			DriverInput[2] = angle; // set rotation input to angle of right stick
			lastJoystickAngle = angle; // update last joystick angle
		} else {
			DriverInput[2] = lastJoystickAngle; // if right stick is not significantly moved
		}

		return DriverInput;
	}
}
