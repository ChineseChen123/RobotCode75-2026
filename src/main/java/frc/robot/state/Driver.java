// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.state;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.DrivetrainConstants.maxAngularVelocity;
import static frc.robot.Constants.DrivetrainConstants.maxVelocity;
import static frc.robot.Constants.DrivetrainConstants.speedClampMultiplier;
import static frc.robot.Constants.DrivetrainConstants.angularClampMultiplier;
import static frc.robot.Constants.IOConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

public class Driver extends SubsystemBase {

	public static final Trigger kFalse = new Trigger(() -> false);

	private final Joystick m_LeftStick;
	private final Joystick m_RightStick;

	public Trigger[] leftButtons = new Trigger[16];
	public Trigger[] rightButtons = new Trigger[16];

	public DoubleSupplier leftX = () -> 0;
	public DoubleSupplier leftY = () -> 0;
	public DoubleSupplier rightX = () -> 0;
	public DoubleSupplier rightY = () -> 0;

	/** Creates a new Driver. */
	public Driver(Joystick leftStick, Joystick rightStick) {
		m_LeftStick = leftStick;
		m_RightStick = rightStick;

		for (int i = 0; i < 16; i++) {
			leftButtons[i] = new JoystickButton(leftStick, i + 1);
			rightButtons[i] = new JoystickButton(rightStick, i + 1);
		}

		leftX =
				() -> {
					double val =
							MathUtil.applyDeadband(m_LeftStick.getX() * translationStickMapValue, stickDeadband);
					return val >= 0
							? Math.pow(val, translationJoystickExpo)
							: -1 * Math.pow(-val, translationJoystickExpo);
				};
		leftY =
				() -> {
					double val =
							MathUtil.applyDeadband(m_LeftStick.getY() * translationStickMapValue, stickDeadband);
					return val >= 0
							? Math.pow(val, translationJoystickExpo)
							: -1 * Math.pow(-val, translationJoystickExpo);
				};
		rightX = () -> MathUtil.applyDeadband(m_RightStick.getX(), stickDeadband);
		rightY = () -> MathUtil.applyDeadband(m_RightStick.getY(), stickDeadband);
	}

	public Trigger getLeftButton(int button) {
		return leftButtons[MathUtil.clamp(button, 1, 16) - 1];
	}

	public Trigger getRightButton(int button) {
		return rightButtons[MathUtil.clamp(button, 1, 16) - 1];
	}

	/** returns array of all 3 processed joystick values (for two drivers) */
	public double[] processedJoystickValues(boolean speedClamp) {
		// Negation because joystick forward is negative
		double[] DriverInput = {-leftY.getAsDouble(), -leftX.getAsDouble(), -rightX.getAsDouble()};
		boolean fieldRelative = RobotContainer.getSwerve().getFieldRelative();
		if (!fieldRelative) {
			DriverInput[0] *= 0.5;
			DriverInput[1] *= 0.5;
			DriverInput[2] *= 0.5;
		} else if (DriverStation.getAlliance().get() == Alliance.Red) {
			DriverInput[0] *= -1;
			DriverInput[1] *= -1;
		}
		DriverInput[0] *= maxVelocity.in(MetersPerSecond) * (speedClamp ? speedClampMultiplier : 1);
		DriverInput[1] *= maxVelocity.in(MetersPerSecond) * (speedClamp ? speedClampMultiplier : 1);
		DriverInput[2] *=
				maxAngularVelocity.in(RadiansPerSecond) * (speedClamp ? angularClampMultiplier : 1);

		return DriverInput;
	}

	double lastJoystickAngle = 0;

	public double[] processedJoystickValuesPositionalRotation(boolean speedClamp) {
		// Negation because joystick forward is negative
		double[] DriverInput = processedJoystickValues(speedClamp);

		// get normalized vector of rotation translation

		double magnitude =
				Math.sqrt(Math.pow(rightX.getAsDouble(), 2) + Math.pow(rightY.getAsDouble(), 2));
		if (magnitude > 0.1) {
			double angle = Math.atan2(rightY.getAsDouble(), rightX.getAsDouble());
			DriverInput[2] = angle; // set rotation input to angle of right stick
			lastJoystickAngle = angle; // update last joystick angle
		} else {
			DriverInput[2] = lastJoystickAngle; // if right stick is not significantly moved
		}

		return DriverInput;
	}
}
