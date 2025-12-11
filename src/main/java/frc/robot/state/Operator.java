// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.state;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

	public DoubleSupplier rightStickX() {
		return () -> m_Controller.getRightX();
	}

	public DoubleSupplier rightStickY() {
		return () -> m_Controller.getRightY();
	}

	public void rumble(double leftIntensity, double rightIntensity) {
		if (!m_Controller.getHID().isConnected()) {
			return;
		}
		m_Controller.getHID().setRumble(RumbleType.kLeftRumble, leftIntensity);
		m_Controller.getHID().setRumble(RumbleType.kRightRumble, rightIntensity);
	}
}
