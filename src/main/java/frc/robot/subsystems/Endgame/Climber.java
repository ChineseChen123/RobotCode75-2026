// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Endgame;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;
import frc.robot.state.RobotStates;

public class Climber extends SubsystemBase {

	// we only have a certain number of states the climber will be in at any given time
	// we also define the positions in rotations here for easy conversion
	public static enum ClimberPositions {
		STOW(stowedPosition),
		UP(upPosition),
		CLIMBED(climbedPosition);

		public final Angle Rotations;

		private ClimberPositions(Angle rotations) {
			this.Rotations = rotations;
		}
	}

	public static enum ClimberState {
		RAISING(raisingVoltage),
		LOWERING(loweringVoltage),
		HOLDING(holdingVoltage),
		SETPOINTUP(raisingVoltage),
		SETPOINTDOWN(loweringVoltage);

		public final Voltage voltage;

		private ClimberState(Voltage voltage) {
			this.voltage = voltage;
		}
	}

	private ClimberPositions m_SetpointPosition = ClimberPositions.STOW;
	private ClimberState m_ClimberState = ClimberState.HOLDING;

	// define motors
	private final TalonFX m_ClimberMotor;

	// define control requests
	private final PositionTorqueCurrentFOC m_PositionRequest;
	private final VoltageOut m_VoltageRequest;

	// private final TunableNumber climberKp;
	// private final TunableNumber climberKi;
	// private final TunableNumber climberKd;
	// private final TunableNumber climberKs;
	// private final TunableNumber climberKa;
	// private final TunableNumber climberKv;
	// private final TunableNumber climberKg;

	private Slot0Configs config;

	public Climber() {
		// initialize motors, using the non drivetrain CANivore bus
		m_ClimberMotor = new TalonFX(climberMotorCANID, superstructureCANBusName);

		m_ClimberState = ClimberState.HOLDING;

		// initialize control requests
		m_PositionRequest = new PositionTorqueCurrentFOC(0);
		m_VoltageRequest = new VoltageOut(0);

		// configure motors with correct inverts
		m_ClimberMotor.getConfigurator().apply(MotorConfigs.getClimberMotorConfig());

		// reset the position of the climber
		m_ClimberMotor.setPosition(ClimberPositions.STOW.Rotations);

		m_PositionRequest.UpdateFreqHz = 0;
		m_PositionRequest.UseTimesync = true;

		m_VoltageRequest.UpdateFreqHz = 0;
		m_VoltageRequest.UseTimesync = true;

		// climberKp = new TunableNumber("Climber/kP", MotorConfigs.kP);
		// climberKi = new TunableNumber("Climber/kI", MotorConfigs.kI);
		// climberKd = new TunableNumber("Climber/kD", MotorConfigs.kD);
		// climberKs = new TunableNumber("Climber/kS", MotorConfigs.kS);
		// climberKa = new TunableNumber("Climber/kA", MotorConfigs.kA);
		// climberKv = new TunableNumber("Climber/kV", MotorConfigs.kV);
		// climberKg = new TunableNumber("Climber/kG", MotorConfigs.kG);

		config =
				new Slot0Configs()
						.withKP(MotorConfigs.kP)
						.withKI(MotorConfigs.kI)
						.withKD(MotorConfigs.kD)
						.withKS(MotorConfigs.kS)
						.withKA(MotorConfigs.kA)
						.withKV(MotorConfigs.kV)
						.withKG(MotorConfigs.kG)
						.withGravityType(GravityTypeValue.Elevator_Static)
						.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

		Timer.delay(5);

		m_ClimberMotor.setPosition(0);
	}

	/**
	 * sets the climber state
	 *
	 * @param position the position to set the climber to
	 */
	public void setPosition(ClimberPositions position) {
		m_SetpointPosition = position;
		// m_ClimberState = ClimberState.SETPOINT;
	}

	/** return position in rotations from home (bottom) */
	public Angle getPosition() {
		return BaseStatusSignal.getLatencyCompensatedValue(
						m_ClimberMotor.getPosition(), m_ClimberMotor.getVelocity());
	}

	/** return current position setpoint */
	@Logged(key = "Climber State", importance = Importance.CRITICAL)
	public ClimberState getState() {
		return m_ClimberState;
	}

	/** position as a double for logging */
	@Logged(key = "Climber Position", importance = Importance.CRITICAL)
	public double logPosition() {
		return getPosition().in(Rotations);
	}

	public boolean isAtPosition(ClimberPositions position) {
		return getPosition().isNear(position.Rotations, climberTolerance);
	}

	/** used to check climber height during auto */
	public boolean isBelowPosition(ClimberPositions position) {
		double currentPosition = getPosition().in(Rotations);
		return currentPosition <= position.Rotations.in(Rotations);
	}

	// raise climber to specified position and hold
	public Command positionCommand(ClimberPositions position) {
		return new InstantCommand(() -> setPosition(position), this)
				.repeatedly()
				.finallyDo(
						() -> {
							m_ClimberMotor.setControl(new StaticBrake());
						});
	}

	// raise climber to specific position and end when at position
	public Command positionCommandUntilDone(ClimberPositions position) {
		return new InstantCommand(() -> setPosition(position), this)
				.repeatedly()
				.until(() -> isAtPosition(position));
	}

	public Command setStateCommand(ClimberState state) {
		return new InstantCommand(() -> m_ClimberState = state, this)
				.repeatedly()
				.finallyDo(() -> m_ClimberState = ClimberState.HOLDING);
	}

	@Override
	public void periodic() {

		// convert climber position to rotations
		double currentPosition = getPosition().in(Rotations);

		double targetRotations =
				m_SetpointPosition.Rotations.in(Rotations); // new pivot subtracts rotations

		// if (config.kP != climberKp.getNumber()
		// 		|| config.kI != climberKi.getNumber()
		// 		|| config.kD != climberKd.getNumber()
		// 		|| config.kS != climberKs.getNumber()
		// 		|| config.kA != climberKa.getNumber()
		// 		|| config.kV != climberKv.getNumber()
		// 		|| config.kG != climberKg.getNumber()) {
		// 	config.kP = climberKp.getNumber();
		// 	config.kI = climberKi.getNumber();
		// 	config.kD = climberKd.getNumber();
		// 	config.kS = climberKs.getNumber();
		// 	config.kA = climberKa.getNumber();
		// 	config.kV = climberKv.getNumber();
		// 	config.kG = climberKg.getNumber();

		// 	m_ClimberMotor1.getConfigurator().apply(config);
		// 	m_ClimberMotor2.getConfigurator().apply(config);
		// }

		if (!RobotStates.teleop.getAsBoolean() || DriverStation.getMatchTime() > 30) {
			return;
		}

		if (m_ClimberState == ClimberState.SETPOINTUP && currentPosition > upPosition.in(Rotations)) {
			m_ClimberState = ClimberState.HOLDING;
		}
		if (m_ClimberState == ClimberState.SETPOINTDOWN && currentPosition < climbedPosition.in(Rotations)) {
			m_ClimberState = ClimberState.HOLDING;
		}

		if (m_ClimberState == ClimberState.HOLDING) {
			m_ClimberMotor.setControl(new StaticBrake());
		} else if (m_ClimberState.voltage != null) {
			m_ClimberMotor.setControl(m_VoltageRequest.withOutput(m_ClimberState.voltage));
		}
	}
}
