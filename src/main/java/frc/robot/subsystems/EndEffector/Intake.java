// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static frc.robot.Constants.EndEffectorConstants.Intake.*;
import static frc.robot.Constants.EndEffectorConstants.Intake.MotorConfigs.*;
import static frc.robot.Constants.RobotConstants.*;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;

public class Intake extends SubsystemBase {

	public static enum IntakeStates {
		STARTING(pivotUpAngle, defaultIntakeSpeed),
		DEFAULT(pivotHalfwayAngle, defaultIntakeSpeed),
		INTAKING(pivotDownAngle, intakeRunningSpeed),
		REVERSING(pivotDownAngle, intakeReversingSpeed);

		Angle pivotPosition;
		AngularVelocity intakeSpeed;

		private IntakeStates(Angle pivotPosition, AngularVelocity intakeSpeed) {
			this.pivotPosition = pivotPosition;
			this.intakeSpeed = intakeSpeed;
		}
	}

	private final TalonFX m_IntakeMotor;
	private final TalonFX m_PivotMotor;

	private final VelocityTorqueCurrentFOC m_IntakeRequest = new VelocityTorqueCurrentFOC(0);
	private final PositionTorqueCurrentFOC m_PivotRequest = new PositionTorqueCurrentFOC(0);

	private IntakeStates m_IntakeState;

	/** Creates a new Intake. */
	public Intake() {
		m_IntakeMotor = new TalonFX(intakeMotorCanID, superstructureCANBusName);
		m_PivotMotor = new TalonFX(pivotCanID, superstructureCANBusName);
		m_PivotMotor.getConfigurator().apply(getPivotConfiguration());
		m_IntakeState = IntakeStates.DEFAULT;

		Timer.delay(5);

		m_PivotMotor.setPosition(0);
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
		m_IntakeMotor.setControl(m_IntakeRequest.withVelocity(m_IntakeState.intakeSpeed));
		m_PivotMotor.setControl(m_PivotRequest.withPosition(m_IntakeState.pivotPosition));
	}
}
