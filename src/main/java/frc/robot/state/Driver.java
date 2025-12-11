// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.state;

import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class Driver extends SubsystemBase {

	public static final Trigger kFalse = new Trigger(() -> false);

	private final Joystick m_LeftStick;
	private final Joystick m_RightStick;

	public Trigger[] leftButtons = new Trigger[16];
	public Trigger[] rightButtons = new Trigger[16];

	/** Creates a new Driver. */
	public Driver(Joystick leftStick, Joystick rightStick) {
		m_LeftStick = leftStick;
		m_RightStick = rightStick;

		for (int i = 0; i < 16; i++) {
			leftButtons[i] = new JoystickButton(leftStick, i + 1);
			rightButtons[i] = new JoystickButton(rightStick, i + 1);
		}
	}

	public Trigger getLeftButton(int button) {
		return leftButtons[MathUtil.clamp(button, 1, 16) - 1];
	}

	public Trigger getRightButton(int button) {
		return rightButtons[MathUtil.clamp(button, 1, 16) - 1];
	}

	public DoubleSupplier leftX() {
		return () -> {
			double val =
					MathUtil.applyDeadband(m_LeftStick.getX() * translationStickMapValue, stickDeadband);
			return val >= 0
					? Math.pow(val, translationJoystickExpo)
					: -1 * Math.pow(-val, translationJoystickExpo);
		};
	}

	public DoubleSupplier leftY() {
		return () -> {
			double val =
					MathUtil.applyDeadband(m_LeftStick.getY() * translationStickMapValue, stickDeadband);
			return val >= 0
					? Math.pow(val, translationJoystickExpo)
					: -1 * Math.pow(-val, translationJoystickExpo);
		};
	}

	public DoubleSupplier rightX() {
		return () -> MathUtil.applyDeadband(m_RightStick.getX(), stickDeadband);
	}

	public DoubleSupplier rightY() {
		return () -> m_RightStick.getY();
	}
}
