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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;

/*
 * Cascading climber driven by 2 Kraken X60s
 * 2 stage WCP GreyT climber
 * Uses motion magic to honor a target \ and acceleration
 *  Motion magic is used to prevent the climber from destroying itself by moving too fast
 *
 */
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

	private ClimberPositions m_SetpointPosition = ClimberPositions.STOW;

	// define motors
	private final TalonFX m_ClimberMotor1;
	private final TalonFX m_ClimberMotor2;

	// define control requests
	private final PositionTorqueCurrentFOC m_PositionRequest;
	private final Follower m_FollowerRequest;

	private final TunableNumber climberKp;
	private final TunableNumber climberKi;
	private final TunableNumber climberKd;
	private final TunableNumber climberKs;
	private final TunableNumber climberKa;
	private final TunableNumber climberKv;
	private final TunableNumber climberKg;

	private Slot0Configs config;

	public Climber() {
		// initialize motors, using the non drivetrain CANivore bus
		m_ClimberMotor1 = new TalonFX(climberMotor1CANID, superstructureCANBusName);
		m_ClimberMotor2 = new TalonFX(climberMotor2CANID, superstructureCANBusName);

		// initialize control requests
		m_PositionRequest = new PositionTorqueCurrentFOC(0);
		m_FollowerRequest = new Follower(climberMotor1CANID, MotorAlignmentValue.Aligned);

		// configure motors with correct inverts
		m_ClimberMotor1.getConfigurator().apply(MotorConfigs.getClimberMotorConfig());
		m_ClimberMotor2.getConfigurator().apply(MotorConfigs.getClimberMotorConfig());

		// reset the position of the climber
		m_ClimberMotor1.setPosition(ClimberPositions.STOW.Rotations);
		m_ClimberMotor2.setPosition(ClimberPositions.STOW.Rotations);

		m_PositionRequest.UpdateFreqHz = 0;
		m_PositionRequest.UseTimesync = true;

		climberKp = new TunableNumber("Climber/kP", MotorConfigs.kP);
		climberKi = new TunableNumber("Climber/kI", MotorConfigs.kI);
		climberKd = new TunableNumber("Climber/kD", MotorConfigs.kD);
		climberKs = new TunableNumber("Climber/kS", MotorConfigs.kS);
		climberKa = new TunableNumber("Climber/kA", MotorConfigs.kA);
		climberKv = new TunableNumber("Climber/kV", MotorConfigs.kV);
		climberKg = new TunableNumber("Climber/kG", MotorConfigs.kG);

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
	}

	/**
	 * sets the climber state
	 *
	 * @param position the position to set the climber to
	 */
	public void setPosition(ClimberPositions position) {
		m_SetpointPosition = position;
	}

	/** return position in rotations from home (bottom) */
	public Angle getPosition() {
		Measure<AngleUnit> motor1Position =
				BaseStatusSignal.getLatencyCompensatedValue(
						m_ClimberMotor1.getPosition(), m_ClimberMotor1.getVelocity());
		Measure<AngleUnit> motor2Position =
				BaseStatusSignal.getLatencyCompensatedValue(
						m_ClimberMotor2.getPosition(), m_ClimberMotor2.getVelocity());

		return Rotations.of((motor1Position.in(Rotations) + motor2Position.in(Rotations)) / 2);
	}

	/** return current position setpoint */
	@Logged(key = "Climber State", importance = Importance.CRITICAL)
	public ClimberPositions getState() {
		return m_SetpointPosition;
	}

	/** position as a double for logging */
	@Logged(key = "Climber Position", importance = Importance.DEBUG)
	public double logPosition() {
		return getPosition().in(Rotations);
	}

	public boolean isAtPosition(ClimberPositions position) {
		if (m_SetpointPosition != position) {
			return false;
		} else {
			return true;
		}
	}

	/** used to check climber height during auto */
	public boolean isBelowPosition(ClimberPositions position, boolean isAlgae) {
		double currentPosition = getPosition().in(Rotations);
		return currentPosition <= position.Rotations.in(Rotations);
	}

	// raise climber to specified position and hold
	public Command positionCommand(ClimberPositions position, boolean isAlgae) {
		return new InstantCommand(() -> setPosition(position), this)
				.repeatedly()
				.finallyDo(
						() -> {
							m_ClimberMotor1.setControl(new StaticBrake());
							m_ClimberMotor2.setControl(new StaticBrake());
						});
	}

	// raise climber to specific position and end when at position
	public Command positionCommandUntilDone(ClimberPositions position) {
		return new InstantCommand(() -> setPosition(position), this)
				.repeatedly()
				.until(() -> isAtPosition(position));
	}

	@Override
	public void periodic() {

		// convert climber position to rotations
		double currentPosition = m_SetpointPosition.Rotations.in(Rotations);

		double targetRotations = currentPosition; // new pivot subtracts rotations

		if (config.kP != climberKp.getNumber()
				|| config.kI != climberKi.getNumber()
				|| config.kD != climberKd.getNumber()
				|| config.kS != climberKs.getNumber()
				|| config.kA != climberKa.getNumber()
				|| config.kV != climberKv.getNumber()
				|| config.kG != climberKg.getNumber()) {
			config.kP = climberKp.getNumber();
			config.kI = climberKi.getNumber();
			config.kD = climberKd.getNumber();
			config.kS = climberKs.getNumber();
			config.kA = climberKa.getNumber();
			config.kV = climberKv.getNumber();
			config.kG = climberKg.getNumber();

			m_ClimberMotor1.getConfigurator().apply(config);
			m_ClimberMotor2.getConfigurator().apply(config);
		}

		m_ClimberMotor1.setControl(m_PositionRequest.withPosition(Rotations.of(targetRotations)));
		m_ClimberMotor2.setControl(m_FollowerRequest);
	}
}
