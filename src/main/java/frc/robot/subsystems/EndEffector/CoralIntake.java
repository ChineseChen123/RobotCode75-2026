// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.EndEffectorConstants.*;
import static frc.robot.Constants.EndEffectorConstants.MotorConfigs.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;
import frc.robot.state.RobotStates;

// import frc.lib.dashboard.TunableNumber;

public class CoralIntake extends SubsystemBase {
	public static enum CoralStates {
		SCOREL1,
		SCOREL23,
		SCOREL4,
		INTAKING,
		POSITIONING, // moving coral forward slightly once detected by beam break
		REVERSING, // reversing back into chute
		DEFAULT // default is when the intake is doing nothing
	}

	private static CoralStates m_CoralIntakeState;

	private TalonFXS m_CoralMotor;

	private DigitalInput m_CoralBeamBreak;

	private final VoltageOut m_CharacterizationRequest;
	private final VelocityVoltage m_VelocityRequest;
	private final PositionVoltage m_PositionRequest;

	private double timeSinceIntaking;

	// tunable numbers
	// private final TunableNumber scoreSpeed;
	// private final TunableNumber rotationsAfterIntake;

	// private final TunableNumber coralVelocitykP;
	// private final TunableNumber coralVelocitykI;
	// private final TunableNumber coralVelocitykD;
	// private final TunableNumber coralVelocitykS;

	// private final TunableNumber coralPositionkP;
	// private final TunableNumber coralPositionkI;
	// private final TunableNumber coralPositionkD;

	// private Slot0Configs velocityConfig = new Slot0Configs();
	// private Slot1Configs positionConfig = new Slot1Configs();

	public CoralIntake() {
		// initialize motors and sensors
		m_CoralMotor = new TalonFXS(coralMotorCanID, superstructureCANBusName);
		m_CoralIntakeState = CoralStates.DEFAULT;
		m_CoralBeamBreak = new DigitalInput(coralBeamBreakPort);

		m_CharacterizationRequest = new VoltageOut(Volts.of(0));
		m_VelocityRequest = new VelocityVoltage(RotationsPerSecond.of(0));
		m_PositionRequest = new PositionVoltage(Rotations.of(0));

		m_CharacterizationRequest.EnableFOC = true;
		m_CharacterizationRequest.UpdateFreqHz = 0;
		m_CharacterizationRequest.UseTimesync = true;

		m_VelocityRequest.EnableFOC = true;
		m_VelocityRequest.UpdateFreqHz = 0;
		m_VelocityRequest.UseTimesync = true;

		m_PositionRequest.EnableFOC = true;
		m_PositionRequest.UpdateFreqHz = 0;
		m_PositionRequest.UseTimesync = true;

		m_CoralMotor.getConfigurator().apply(getCoralMotorConfiguration());

		timeSinceIntaking = Timer.getFPGATimestamp();

		// scoreSpeed =
		//     new TunableNumber("Coral Intake/Score Speed", coralScoreSpeed.in(RotationsPerSecond));
		// rotationsAfterIntake =
		//     new TunableNumber(
		//         "Coral Intake/Rotations After Intake", coralRotationsAfterIntake.in(Rotations));

		// coralVelocitykP = new TunableNumber("Coral Intake/Velocity Kp", coralVelocityKP);
		// coralVelocitykI = new TunableNumber("Coral Intake/Velocity Ki", coralVelocityKI);
		// coralVelocitykD = new TunableNumber("Coral Intake/Velocity Kd", coralVelocityKD);
		// coralVelocitykS = new TunableNumber("Coral Intake/Velocity Ks", coralVelocityKS);

		// coralPositionkP = new TunableNumber("Coral Intake/Position Kp", coralPositionKP);
		// coralPositionkI = new TunableNumber("Coral Intake/Position Ki", coralPositionKI);
		// coralPositionkD = new TunableNumber("Coral Intake/Position Kd", coralPositionKD);

	}

	/** set coral intake state */
	public void setState(CoralStates state) {
		m_CoralIntakeState = state;
	}

	/** return coral intake state */
	@Logged(key = "Coral State", importance = Importance.CRITICAL)
	public CoralStates getState() {
		return m_CoralIntakeState;
	}

	/** return whether robot has coral */
	@Logged(key = "Has Coral", importance = Importance.CRITICAL)
	public boolean hasCoral() {
		return !m_CoralBeamBreak.get();
	}

	/** set motor encoder to specified position */
	public void resetPosition(Angle angle) {
		m_CoralMotor.setPosition(angle);
	}

	/** return whether motor is done positioning */
	public boolean atPosition() {
		return m_CoralMotor.getPosition(true).getValue().in(Rotations)
				> coralRotationsAfterIntake.in(Rotations);
	}

	/** return speed in rotations per second */
	public double getVelocity() {
		return m_CoralMotor.getVelocity(true).getValue().in(RotationsPerSecond);
	}

	/** return position in rotations */
	public double getPosition() {
		return m_CoralMotor.getPosition(true).getValue().in(Rotations);
	}

	/** return how long coral intake has been continuously intaking for */
	public boolean hasBeenIntakingForTime(double seconds) {
		return (Timer.getFPGATimestamp() - timeSinceIntaking) > seconds
				&& getState() == CoralStates.INTAKING;
	}

	/** set state until interrupted (non-persistent) */
	public Command setStateCommand(CoralStates state) {
		return new InstantCommand(() -> setState(state), this)
				.repeatedly()
				.finallyDo(() -> setState(CoralStates.DEFAULT));
	}

	/** entire intaking sequence (intaking + positioning) */
	public Command intakeCommand() {
		return Commands.sequence(
						new InstantCommand(() -> setState(CoralStates.INTAKING), this),
						new WaitUntilCommand(RobotStates.hasCoral),
						new InstantCommand(
								() -> {
									setState(CoralStates.POSITIONING);
									resetPosition(Rotations.of(0));
								},
								this),
						new WaitUntilCommand(() -> atPosition()))
				.finallyDo(() -> setState(CoralStates.DEFAULT));
	}

	/** update time coral intake has been intaking for */
	public Command updateTimerCommand() {
		return new InstantCommand(
						() -> {
							timeSinceIntaking = Timer.getFPGATimestamp(); // todo possibly rework for autos?
						})
				.repeatedly();
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("Has Coral", hasCoral());

		//     if (coralVelocitykP.getNumber() != velocityConfig.kP
		//     || coralVelocitykI.getNumber() != velocityConfig.kI
		//     || coralVelocitykD.getNumber() != velocityConfig.kD
		//     || coralVelocitykS.getNumber() != velocityConfig.kS) {
		//       velocityConfig.kP = coralVelocitykP.getNumber();
		//       velocityConfig.kD = coralVelocitykD.getNumber();
		//       velocityConfig.kS = coralVelocitykS.getNumber();

		//   m_CoralMotor.getConfigurator().apply(velocityConfig);
		// }

		// if (coralPositionkP.getNumber() != positionConfig.kP
		// || coralPositionkI.getNumber() != positionConfig.kI
		// || coralPositionkD.getNumber() != positionConfig.kD) {
		//   positionConfig.kP = coralPositionkP.getNumber();
		//   positionConfig.kI = coralPositionkI.getNumber();
		//   positionConfig.kD = coralPositionkD.getNumber();

		// m_CoralMotor.getConfigurator().apply(positionConfig);
		// }

		// speed multiplier for L4
		double speedMultiplier = 1.81;

		// set output based on state
		switch (m_CoralIntakeState) {
			case SCOREL1 -> {
				m_CoralMotor.setControl(m_VelocityRequest.withVelocity(coralScoreSpeedL1).withSlot(0));
			}
			case SCOREL23 -> {
				m_CoralMotor.setControl(m_VelocityRequest.withVelocity(coralScoreSpeed).withSlot(0));
			}
			case SCOREL4 -> {
				m_CoralMotor.setControl(
						m_VelocityRequest.withVelocity(coralScoreSpeed.times(speedMultiplier)).withSlot(0));
			}
			case INTAKING -> {
				m_CoralMotor.setControl(
						m_VelocityRequest
								.withVelocity(
										coralIntakeSpeed.times(RobotStates.auto.getAsBoolean() ? speedMultiplier : 1))
								.withSlot(0));
			}
			case POSITIONING -> {
				m_CoralMotor.setControl(
						m_PositionRequest
								.withPosition(coralRotationsAfterIntake.in(Rotations) * coralMotorGearRatio)
								.withSlot(1));
			}
			case REVERSING -> {
				m_CoralMotor.setControl(m_VelocityRequest.withVelocity(coralReverseSpeed).withSlot(0));
			}
			case DEFAULT -> {
				m_CoralMotor.setControl(m_CharacterizationRequest.withOutput(0));
			}
		}
	}
}
