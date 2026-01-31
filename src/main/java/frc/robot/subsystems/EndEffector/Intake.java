// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.EndEffectorConstants.Intake.*;
import static frc.robot.Constants.EndEffectorConstants.Intake.MotorConfigs.*;
import static frc.robot.Constants.RobotConstants.*;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;

public class Intake extends SubsystemBase {

	public static enum IntakeStates {
		DEFAULT(pivotUpAngle, RotationsPerSecond.of(0)),
		DOWN(pivotDownAngle, RotationsPerSecond.of(0)),
		INTAKING(pivotDownAngle, intakeSpeed),
		REVERSING(pivotDownAngle, reverseSpeed);

		Angle angle;
		AngularVelocity speed;

		private IntakeStates(Angle angle, AngularVelocity speed) {
			this.angle = angle;
			this.speed = speed;
		}
	}

	private final TalonFX m_IntakeMotor;
	private final TalonFX m_PivotMotor;

	private final DutyCycleEncoder m_absoluteEncoder;

	private final VelocityTorqueCurrentFOC m_IntakeRequest = new VelocityTorqueCurrentFOC(0);
	private final PositionTorqueCurrentFOC m_PivotRequest = new PositionTorqueCurrentFOC(0);

	private IntakeStates m_IntakeState;

	public Intake() {
		m_IntakeMotor = new TalonFX(intakeMotorCanID, superstructureCANBusName);
		m_PivotMotor = new TalonFX(pivotCanID, superstructureCANBusName);

		m_absoluteEncoder = new DutyCycleEncoder(pivotEncoderPort, 1, pivotZeroPoint.in(Rotations));

		m_PivotMotor.getConfigurator().apply(getPivotConfiguration());
		m_IntakeState = IntakeStates.DEFAULT;

		Timer.delay(5);
		m_PivotMotor.setPosition(
				(getAbsolutePosition() - pivotEncoderOffset.in(Rotations)) * pivotGearRatio);
	}

	public void resetPivotMotor(Angle rotations) {
		m_PivotMotor.setPosition(rotations);
	}

	public double getAbsolutePosition() {
		return m_absoluteEncoder.get();
	}

	public double getPivotPosition() {
		return m_PivotMotor.getPosition().refresh().getValue().in(Rotations);
	}

	public boolean isAtPositionAbsolute(double absolutePosition) {
		return Math.abs(absolutePosition - m_absoluteEncoder.get()) < pivotToleranceAbsolute;
	}

	public boolean isAtPosition(IntakeStates state) {
		return Math.abs(state.angle.in(Rotations) - m_PivotMotor.getPosition().getValue().in(Rotations))
				< pivotToleranceAbsolute;
	}

	public double getIntakeVelocity() {
		return m_IntakeMotor.getVelocity(true).getValue().in(RotationsPerSecond);
	}

	@Logged(key = "Intake State", importance = Importance.CRITICAL)
	public IntakeStates getIntakeState() {
		return m_IntakeState;
	}

	public void setState(IntakeStates state) {
		m_IntakeState = state;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		m_PivotMotor.setControl(m_PivotRequest.withPosition(m_IntakeState.angle));
		m_IntakeMotor.setControl(m_IntakeRequest.withVelocity(m_IntakeState.speed));
	}
}
