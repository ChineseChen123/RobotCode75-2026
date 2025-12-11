// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ClimberConstants.MotorConfigs.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

	private final TalonFX m_ClimberMotor1;
	private final TalonFX m_ClimberMotor2;

	private final MotionMagicExpoTorqueCurrentFOC m_PositionRequest;
	private final TorqueCurrentFOC m_CurrentRequest;

	private final DigitalInput m_ClimberLimitSwitch;

	private boolean isClimberReset = false;

	public Climber() {
		// Initialize motors and limit switch
		m_ClimberMotor1 = new TalonFX(ClimberConstants.climberMotor1CANID, superstructureCANBusName);
		m_ClimberMotor2 = new TalonFX(ClimberConstants.climberMotor2CANID, superstructureCANBusName);
		m_PositionRequest = new MotionMagicExpoTorqueCurrentFOC(0);
		m_CurrentRequest = new TorqueCurrentFOC(Amps.of(0));

		m_ClimberLimitSwitch = new DigitalInput(8);

		m_ClimberMotor1.getConfigurator().apply(getClimberMotorConfig());
		m_ClimberMotor2.getConfigurator().apply(getClimberMotorConfig());

		resetPosition();
	}

	// Get limit switch value (normally open)
	public boolean getLimitSwitch() {
		return m_ClimberLimitSwitch.get();
	}

	// Convert throughbore encoder value to motor rotations
	public double absoluteEncoderToRotations(double x) {
		return 132.2772 * Math.sin(3.36922 * x) + 11.17;
	}

	// Set climber target position if limit not triggered
	public void setPositionRequest(Angle position) {
		if ((getLimitSwitch() || atPosition(position))
				&& position.in(Rotations) <= getPositionRotations()) {
			m_ClimberMotor1.setControl(m_CurrentRequest.withOutput(0));
			m_ClimberMotor2.setControl(m_CurrentRequest.withOutput(0));
		} else {
			m_ClimberMotor1.setControl(
					m_PositionRequest.withPosition(position).withLimitReverseMotion(getLimitSwitch()));
			m_ClimberMotor2.setControl(
					m_PositionRequest.withPosition(position).withLimitReverseMotion(getLimitSwitch()));
		}
	}

	public double getPositionRotations() {
		return getPosition().in(Rotations);
	}

	// Average of each motor position
	public Angle getPosition() {
		Measure<AngleUnit> motor1Position =
				BaseStatusSignal.getLatencyCompensatedValue(
						m_ClimberMotor1.getPosition(true), m_ClimberMotor1.getVelocity(true));
		Measure<AngleUnit> motor2Position =
				BaseStatusSignal.getLatencyCompensatedValue(
						m_ClimberMotor2.getPosition(true), m_ClimberMotor2.getVelocity(true));

		return Rotations.of((motor1Position.in(Rotations) + motor2Position.in(Rotations)) / 2);
	}

	public boolean atPosition(Angle position) {
		return Math.abs(getPosition().in(Rotations) - position.in(Rotations))
				< ClimberConstants.climbPositionTolerance;
	}

	// Run climber at a given current, disregarding position
	// Current value: negative inward, positive outward
	public void runCurrent(double current) {
		current = MathUtil.applyDeadband(current, 5);

		if (getLimitSwitch() && current < 0) {
			current = 0;
		}

		m_ClimberMotor1.setControl(m_CurrentRequest.withOutput(current));
		m_ClimberMotor2.setControl(m_CurrentRequest.withOutput(current));
	}

	// Reset motor encoder position if limit is triggered
	public void resetPosition() {
		if (getLimitSwitch()) {
			m_ClimberMotor1.setPosition(0);
			m_ClimberMotor2.setPosition(0);
			isClimberReset = true;
		}
	}

	public boolean isReset() {
		return isClimberReset;
	}

	@Override
	public void periodic() {
		// Reset if significant error in motor encoder value
		if (getLimitSwitch() && Math.abs(getPosition().in(Rotations)) >= 0.1) {
			resetPosition();
		}
		SmartDashboard.putBoolean("Climber Limit", getLimitSwitch());

		// if (climberKp.getNumber() != PIDConfig.kP
		//     || climberKd.getNumber() != PIDConfig.kD
		//     || climberKs.getNumber() != PIDConfig.kS
		//     || climberKg.getNumber() != PIDConfig.kG
		//     || climberKa.getNumber() != PIDConfig.kA
		//     || climberKv.getNumber() != PIDConfig.kV) {

		//   PIDConfig.kP = climberKp.getNumber();
		//   PIDConfig.kD = climberKd.getNumber();
		//   PIDConfig.kS = climberKs.getNumber();
		//   PIDConfig.kG = climberKg.getNumber();
		//   PIDConfig.kA = climberKa.getNumber();
		//   PIDConfig.kV = climberKv.getNumber();

		//   m_ClimberMotor1.getConfigurator().apply(PIDConfig);
		//   m_ClimberMotor2.getConfigurator().apply(PIDConfig);
		// }
		// if (climberMMCruiseVelocity.getNumber() != MMConfig.MotionMagicCruiseVelocity
		//     || climberMMCruiseAcceleration.getNumber() != MMConfig.MotionMagicAcceleration
		//     || climberMMKv.getNumber() != MMConfig.MotionMagicExpo_kV
		//     || climberMMKa.getNumber() != MMConfig.MotionMagicExpo_kA) {

		//   MMConfig.MotionMagicCruiseVelocity = climberMMCruiseVelocity.getNumber();
		//   MMConfig.MotionMagicAcceleration = climberMMCruiseAcceleration.getNumber();
		//   MMConfig.MotionMagicExpo_kV = climberMMKv.getNumber();
		//   MMConfig.MotionMagicExpo_kA = climberMMKa.getNumber();

		//   m_ClimberMotor1.getConfigurator().apply(MMConfig);
		//   m_ClimberMotor2.getConfigurator().apply(MMConfig);
		// }
	}
}
