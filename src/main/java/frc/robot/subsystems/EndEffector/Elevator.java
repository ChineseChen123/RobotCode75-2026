// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.ElevatorConstants.MotorConfigs.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;

/*
 * Cascading elevator driven by 2 Kraken X60s
 * 2 stage WCP GreyT elevator
 * Uses motion magic to honor a target \ and acceleration
 *  Motion magic is used to prevent the elevator from destroying itself by moving too fast
 *
 */
public class Elevator extends SubsystemBase {

	// we only have a certain number of states the elevator will be in at any given time
	// we also define the positions in rotations here for easy conversion
	public static enum ElevatorPositions {
		L1(l1Position),
		L2(l2Position),
		L3(l3Position),
		L4(l4Position),
		NET(netPosition),
		HOME(homePosition),
		PROCESSOR(processorPosition);

		public final Angle Rotations;

		private ElevatorPositions(Angle rotations) {
			this.Rotations = rotations;
		}
	}

	private ElevatorPositions m_SetpointPosition = ElevatorPositions.HOME;
	// is our current setpoint for dealgaefying
	private boolean m_IsAlgae = false;

	// define motors
	private final TalonFX m_ElevatorMotor1;
	private final TalonFX m_ElevatorMotor2;

	// sensors
	private final DigitalInput m_lowerLimitSwitch;
	private final DigitalInput m_upperLimitSwitch;
	private final DigitalInput m_backupLimitSwitch;

	// define control requests
	private final DynamicMotionMagicTorqueCurrentFOC m_PositionRequest;
	private final TorqueCurrentFOC m_CharacterizationRequest;

	// tunable numbers
	// private final TunableNumber mmVelocityUp;
	// private final TunableNumber mmAccelerationUp;
	// private final TunableNumber mmJerkUp;

	// private final TunableNumber mmVelocityDown;
	// private final TunableNumber mmAccelerationDown;
	// private final TunableNumber mmJerkDown;

	// private final TunableNumber elevatorKp;
	// private final TunableNumber elevatorKi;
	// private final TunableNumber elevatorKd;
	// private final TunableNumber elevatorKs;
	// private final TunableNumber elevatorKa;
	// private final TunableNumber elevatorKv;
	// private final TunableNumber elevatorKg;

	private Slot0Configs config;

	public Elevator() {
		// initialize motors, using the non drivetrain CANivore bus
		m_ElevatorMotor1 = new TalonFX(elevatorMotor1CANID, superstructureCANBusName);
		m_ElevatorMotor2 = new TalonFX(elevatorMotor2CANID, superstructureCANBusName);

		// initialize sensors
		m_lowerLimitSwitch = new DigitalInput(lowerLimitPort);
		m_upperLimitSwitch = new DigitalInput(upperLimitPort);
		m_backupLimitSwitch = new DigitalInput(backupLimitPort);

		// initialize control requests
		m_PositionRequest = new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0, 0);
		m_CharacterizationRequest = new TorqueCurrentFOC(Amps.of(0));

		// configure motors with correct inverts
		m_ElevatorMotor1.getConfigurator().apply(getElevatorMotorConfig());
		m_ElevatorMotor2.getConfigurator().apply(getElevatorMotorConfig());

		// reset the position of the elevator
		m_ElevatorMotor1.setPosition(ElevatorPositions.HOME.Rotations);
		m_ElevatorMotor2.setPosition(ElevatorPositions.HOME.Rotations);

		m_CharacterizationRequest.UpdateFreqHz = 0;
		m_CharacterizationRequest.UseTimesync = true;

		m_PositionRequest.UpdateFreqHz = 0;
		m_PositionRequest.UseTimesync = true;

		// mmVelocityUp = new TunableNumber("Elevator/MM Velocity Up", MotionMagicProfileUp[0]);
		// mmAccelerationUp = new TunableNumber("Elevator/MM Accleration Up", MotionMagicProfileUp[1]);
		// mmJerkUp = new TunableNumber("Elevator/MM Jerk Up", MotionMagicProfileUp[2]);

		// mmVelocityDown = new TunableNumber("Elevator/MM Velocity Down", MotionMagicProfileDown[0]);
		// mmAccelerationDown =
		//     new TunableNumber("Elevator/MM Acceleration Down", MotionMagicProfileDown[1]);
		// mmJerkDown = new TunableNumber("Elevator/MM Jerk Down", MotionMagicProfileDown[2]);

		// elevatorKp = newTunableNumber("Elevator/kP", kP);
		// elevatorKi = new TunableNumber("Elevator/kI", kI);
		// elevatorKd = new TunableNumber("Elevator/kD", kD);
		// elevatorKs = new TunableNumber("Elevator/kS", kS);
		// elevatorKa = new TunableNumber("Elevator/kA", kA);
		// elevatorKv = new TunableNumber("Elevator/kV", kV);
		// elevatorKg = new TunableNumber("Elevator/kG", kG);

		config =
				new Slot0Configs()
						.withKP(kP)
						.withKI(kI)
						.withKD(kD)
						.withKS(kS)
						.withKA(kA)
						.withKV(kV)
						.withKG(kG)
						.withGravityType(GravityTypeValue.Elevator_Static)
						.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
	}

	/**
	 * sets the elevator state
	 *
	 * @param position the position to set the elevator to
	 */
	public void setPosition(ElevatorPositions position, boolean isAlgae) {
		m_SetpointPosition = position;
		m_IsAlgae = isAlgae;
	}

	/** return position in rotations from home (bottom) */
	public Angle getPosition() {
		Measure<AngleUnit> motor1Position =
				BaseStatusSignal.getLatencyCompensatedValue(
						m_ElevatorMotor1.getPosition(), m_ElevatorMotor1.getVelocity());
		Measure<AngleUnit> motor2Position =
				BaseStatusSignal.getLatencyCompensatedValue(
						m_ElevatorMotor2.getPosition(), m_ElevatorMotor2.getVelocity());

		return Rotations.of((motor1Position.in(Rotations) + motor2Position.in(Rotations)) / 2);
	}

	/** return current position setpoint */
	@Logged(key = "Elevator State", importance = Importance.CRITICAL)
	public ElevatorPositions getState() {
		return m_SetpointPosition;
	}

	/** position as a double for logging */
	@Logged(key = "Elevator Position", importance = Importance.DEBUG)
	public double logPosition() {
		return getPosition().in(Rotations);
	}

	public boolean atL4() {
		return isAtPosition(ElevatorPositions.L4, false);
	}

	public boolean isAtPosition(ElevatorPositions position, boolean isAlgae) {
		if (m_SetpointPosition != position || m_IsAlgae != isAlgae) {
			return false;
		}
		if (position == ElevatorPositions.HOME) {
			return getLowerLimit();
		}
		double currentPosition = getPosition().in(Rotations);
		// problem with algaeOffset in []\isAtpoisitoin
		double algaeOffset =
				(isAlgae && (position == ElevatorPositions.L2 || position == ElevatorPositions.L3))
						? algaeRemovalOffset.in(Rotations)
						// ? AlgaeOffsetPrePickupRotations.getNumber()
						: 0;
		double actualDeadband =
				elevatorTolerance.in(Rotations) + (position == ElevatorPositions.L4 ? +0.5 : 0);
		return MathUtil.applyDeadband(
						currentPosition - (position.Rotations.in(Rotations) - algaeOffset), actualDeadband)
				== 0.0;
	}

	/** used to check elevator height during auto */
	public boolean isBelowPosition(ElevatorPositions position, boolean isAlgae) {
		if (position == ElevatorPositions.HOME) {
			return getLowerLimit();
		}
		double currentPosition = getPosition().in(Rotations);
		double algaeOffset =
				(isAlgae && (position == ElevatorPositions.L2 || position == ElevatorPositions.L3))
						? algaeRemovalOffset.in(Rotations)
						// ? AlgaeOffsetPrePickupRotations.getNumber()
						: 0;
		currentPosition -= algaeOffset;
		return currentPosition <= position.Rotations.in(Rotations);
	}

	/** Limits: pressed = true */
	public boolean getUpperLimit() {
		return !m_upperLimitSwitch.get();
	}

	public boolean getLowerLimit() {
		return !m_lowerLimitSwitch.get(); // || !m_backupLimitSwitch.get();
	}

	public boolean getLowerLimitOne() {
		return !m_lowerLimitSwitch.get();
	}

	public boolean getLowerLimitTwo() {
		return !m_backupLimitSwitch.get();
	}

	/** convert motor rotations to linear vertical inches */
	private Distance rotationsToInches(Angle rotations) {
		return Inches.of(rotations.in(Rotations) * inchesPerRotation.in(Inches));
	}

	// raise elevator to specified position and hold
	public Command positionCommand(ElevatorPositions position, boolean isAlgae) {
		return new InstantCommand(() -> setPosition(position, isAlgae), this)
				.repeatedly()
				.finallyDo(
						() -> {
							m_ElevatorMotor1.setControl(m_CharacterizationRequest.withOutput(0));
							m_ElevatorMotor2.setControl(m_CharacterizationRequest.withOutput(0));
						});
	}

	// raise elevator to specific position and end when at position
	public Command positionCommandUntilDone(ElevatorPositions position, boolean isAlgae) {
		return new InstantCommand(() -> setPosition(position, isAlgae), this)
				.repeatedly()
				.until(() -> isAtPosition(position, isAlgae));
	}

	@Override
	public void periodic() {
		// reset position if at limits
		if (getLowerLimit()
				&& (getPosition().in(Rotations) >= 0.1 || getPosition().in(Rotations) <= -0.1)) {
			m_ElevatorMotor1.setPosition(Rotations.of(0));
			m_ElevatorMotor2.setPosition(Rotations.of(0));
		}
		if (getUpperLimit()) {
			m_ElevatorMotor1.setPosition(Rotations.of(26));
			m_ElevatorMotor2.setPosition(Rotations.of(26));
		}

		// convert elevator position to rotations
		double currentPosition = m_SetpointPosition.Rotations.in(Rotations);

		// calculate how much lower setpoint has to be for dealgaefying
		double algaeOffset =
				(m_IsAlgae
								&& (m_SetpointPosition == ElevatorPositions.L2
										|| m_SetpointPosition == ElevatorPositions.L3))
						? algaeRemovalOffset.in(Rotations)
						: 0;

		double targetRotations = currentPosition - algaeOffset; // new pivot subtracts rotations

		double multiplier = 1;
		if (DriverStation.isAutonomous()) {
			multiplier = 1.1;
		}

		// Change motion profile based on direction of travel
		if (getPosition().in(Rotations) < targetRotations) {
			m_PositionRequest.Velocity = MotionMagicProfileUp[0] * multiplier;
			m_PositionRequest.Acceleration = MotionMagicProfileUp[1] * multiplier;
			m_PositionRequest.Jerk = MotionMagicProfileUp[2];

			// m_PositionRequest.Velocity = mmVelocityUp.getNumber();
			// m_PositionRequest.Acceleration = mmAccelerationUp.getNumber();
			// m_PositionRequest.Jerk = mmJerkUp.getNumber();
		} else {
			m_PositionRequest.Velocity = MotionMagicProfileDown[0] * multiplier;
			m_PositionRequest.Acceleration = MotionMagicProfileDown[1] * multiplier;
			m_PositionRequest.Jerk = MotionMagicProfileDown[2];
			// m_PositionRequest.Velocity = mmVelocityDown.getNumber();
			// m_PositionRequest.Acceleration = mmAccelerationDown.getNumber();
			// m_PositionRequest.Jerk = mmJerkDown.getNumber();
		}

		// if (config.kP != elevatorKp.getNumber()
		// 		|| config.kI != elevatorKi.getNumber()
		// 		|| config.kD != elevatorKd.getNumber()
		// 		|| config.kS != elevatorKs.getNumber()
		// 		|| config.kA != elevatorKa.getNumber()
		// 		|| config.kV != elevatorKv.getNumber()
		// 		|| config.kG != elevatorKg.getNumber()) {
		// 	config.kP = elevatorKp.getNumber();
		// 	config.kI = elevatorKi.getNumber();
		// 	config.kD = elevatorKd.getNumber();
		// 	config.kS = elevatorKs.getNumber();
		// 	config.kA = elevatorKa.getNumber();
		// 	config.kV = elevatorKv.getNumber();
		// 	config.kG = elevatorKg.getNumber();

		// 	m_ElevatorMotor1.getConfigurator().apply(config);
		// 	m_ElevatorMotor2.getConfigurator().apply(config);
		// }

		// set motor outputs
		if (getLowerLimit() && m_SetpointPosition == ElevatorPositions.HOME) {
			m_ElevatorMotor1.setControl(m_CharacterizationRequest.withOutput(0));
			m_ElevatorMotor2.setControl(m_CharacterizationRequest.withOutput(0));
		} else {
			m_ElevatorMotor1.setControl(
					m_PositionRequest
							.withPosition(Rotations.of(targetRotations))
							.withLimitForwardMotion(getUpperLimit())
							.withLimitReverseMotion(getLowerLimit()));
			m_ElevatorMotor2.setControl(
					m_PositionRequest
							.withPosition(Rotations.of(targetRotations))
							.withLimitForwardMotion(getUpperLimit())
							.withLimitReverseMotion(getLowerLimit()));
		}
	}
}
