// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.IntakeIndexConstants.IntakeConstants.*;
import static frc.robot.Constants.IntakeIndexConstants.IntakeConstants.MotorConfigs.*;
import static frc.robot.Constants.RobotConstants.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;
import frc.robot.state.RobotStates;

public class Intake extends SubsystemBase {

	// ── State enum ───────────────────────────────────────────────────────────────

	public static enum IntakeStates {
		STOWED(pivotUpAngle, defaultIntakeSpeed),
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

	// ── Hardware ─────────────────────────────────────────────────────────────────

	private final TalonFX m_IntakeMotor;
	private final TalonFX m_PivotMotor;
	private final DutyCycleEncoder m_absoluteEncoder;

	// ── Control requests ─────────────────────────────────────────────────────────

	private final VelocityTorqueCurrentFOC m_IntakeRequest = new VelocityTorqueCurrentFOC(0);
	private final MotionMagicExpoTorqueCurrentFOC m_PivotRequest =
			new MotionMagicExpoTorqueCurrentFOC(0);

	// ── Internal state ───────────────────────────────────────────────────────────

	private IntakeStates m_IntakeState;

	// ── Pivot tunables / configs ────────────────────────────────────────────────

	private Slot0Configs PivotPIDConfig = new Slot0Configs();

	private final TunableNumber intakePivotKp;
	private final TunableNumber intakePivotKd;
	private final TunableNumber intakePivotKs;
	private final TunableNumber intakePivotKg;

	private MotionMagicConfigs PivotMMConfigs = new MotionMagicConfigs();

	private final TunableNumber intakePivotMMAcc;
	private final TunableNumber intakePivotMMVel;
	private final TunableNumber intakePivotMMJerk;
	private final TunableNumber intakePivotMMKa;
	private final TunableNumber intakePivotMMKv;

	// ── Intake motor tunables / configs ─────────────────────────────────────────

	private Slot0Configs IntakeMotorPIDConfig = new Slot0Configs();

	private final TunableNumber intakeMotorKp;
	private final TunableNumber intakeMotorKi;
	private final TunableNumber intakeMotorKd;
	private final TunableNumber intakeMotorKs;

	/** Creates a new Intake. */
	public Intake() {
		m_IntakeMotor = new TalonFX(intakeMotorCanID, superstructureCANBusName);
		m_PivotMotor = new TalonFX(pivotCanID, superstructureCANBusName);

		m_PivotMotor.getConfigurator().apply(getPivotConfiguration());
		m_IntakeMotor.getConfigurator().apply(getIntakeBangBangConfiguration());

		m_IntakeState = IntakeStates.DEFAULT;

		PivotMMConfigs.withMotionMagicAcceleration(pivotMMAcc)
				.withMotionMagicCruiseVelocity(pivotMMVel)
				.withMotionMagicJerk(pivotMMJerk)
				.withMotionMagicExpo_kA(pivotMMKa)
				.withMotionMagicExpo_kV(pivotMMKv);

		intakePivotMMAcc = new TunableNumber("Intake Pivot/MMAcc", pivotMMAcc);
		intakePivotMMVel = new TunableNumber("Intake Pivot/MMVel", pivotMMVel);
		intakePivotMMJerk = new TunableNumber("Intake Pivot/MMJerk", pivotMMJerk);
		intakePivotMMKa = new TunableNumber("Intake Pivot/MMKa", pivotMMKa);
		intakePivotMMKv = new TunableNumber("Intake Pivot/MMKv", pivotMMKv);

		PivotPIDConfig.withKS(pivotKS)
				.withKG(pivotKG)
				.withKP(pivotKP)
				.withKD(pivotKD)
				.withGravityType(GravityTypeValue.Arm_Cosine);

		intakePivotKp = new TunableNumber("Intake Pivot/kP", pivotKP);
		intakePivotKd = new TunableNumber("Intake Pivot/kD", pivotKD);
		intakePivotKg = new TunableNumber("Intake Pivot/kG", pivotKG);
		intakePivotKs = new TunableNumber("Intake Pivot/kS", pivotKS);

		IntakeMotorPIDConfig.withKP(intakeVelocityKP)
				.withKI(intakeVelocityKI)
				.withKD(intakeVelocityKD)
				.withKS(intakeVelocityKS);

		intakeMotorKp = new TunableNumber("Intake Motor/kP", intakeVelocityKP);
		intakeMotorKi = new TunableNumber("Intake Motor/kI", intakeVelocityKI);
		intakeMotorKd = new TunableNumber("Intake Motor/kD", intakeVelocityKD);
		intakeMotorKs = new TunableNumber("Intake Motor/kS", intakeVelocityKS);

		m_absoluteEncoder = new DutyCycleEncoder(pivotEncoderPort, 1, pivotZeroPoint.in(Rotations));

		// Reset position after a short delay
		Timer.delay(5);
		m_PivotMotor.setPosition((pivotEncoderOffset.in(Rotations) - getThroughborePosition()));
	}

	// ── Sensor / state accessors ────────────────────────────────────────────────

	/** Return through-bore encoder position. */
	@Logged(key = "Abs Encoder Position", importance = Importance.CRITICAL)
	public double getThroughborePosition() {
		return m_absoluteEncoder.get();
	}

	@Logged(key = "Pivot Motor Rotations", importance = Importance.CRITICAL)
	public double getMotorRotations() {
		return m_PivotMotor.getPosition().getValue().in(Rotations);
	}

	@Logged(key = "Intake Velocity", importance = Importance.CRITICAL)
	public double getIntakeVelocity() {
		return m_IntakeMotor.getVelocity().getValue().in(RotationsPerSecond);
	}

	@Logged(key = "Intake State", importance = Importance.CRITICAL)
	public IntakeStates getIntakeState() {
		return m_IntakeState;
	}

	public boolean isAtPosition(IntakeStates state) {
		return Math.abs(state.pivotPosition.in(Rotations) - getMotorRotations())
				< pivotToleranceAbsolute;
	}

	public void setState(IntakeStates state) {
		m_IntakeState = state;
	}

	public Command setStateCommand(IntakeStates state) {
		return new InstantCommand(() -> setState(state), this)
				.repeatedly()
				.finallyDo(() -> setState(IntakeStates.DEFAULT));
	}

	@Override
	public void periodic() {
		// if (intakePivotKp.getNumber() != PivotPIDConfig.kP
		// 		|| intakePivotKd.getNumber() != PivotPIDConfig.kD
		// 		|| intakePivotKs.getNumber() != PivotPIDConfig.kS
		// 		|| intakePivotKg.getNumber() != PivotPIDConfig.kG) {
		// 	PivotPIDConfig.kP = intakePivotKp.getNumber();
		// 	PivotPIDConfig.kD = intakePivotKd.getNumber();
		// 	PivotPIDConfig.kS = intakePivotKs.getNumber();
		// 	PivotPIDConfig.kG = intakePivotKg.getNumber();

		// 	m_PivotMotor.getConfigurator().apply(PivotPIDConfig);
		// }

		// if (intakePivotMMAcc.getNumber() != PivotMMConfigs.MotionMagicAcceleration
		// 		|| intakePivotMMVel.getNumber() != PivotMMConfigs.MotionMagicCruiseVelocity
		// 		|| intakePivotMMJerk.getNumber() != PivotMMConfigs.MotionMagicJerk
		// 		|| intakePivotMMKv.getNumber() != PivotMMConfigs.MotionMagicExpo_kV
		// 		|| intakePivotMMKa.getNumber() != PivotMMConfigs.MotionMagicExpo_kA) {
		// 	PivotMMConfigs.MotionMagicAcceleration = intakePivotMMAcc.getNumber();
		// 	PivotMMConfigs.MotionMagicCruiseVelocity = intakePivotMMVel.getNumber();
		// 	PivotMMConfigs.MotionMagicJerk = intakePivotMMJerk.getNumber();
		// 	PivotMMConfigs.MotionMagicExpo_kV = intakePivotMMKv.getNumber();
		// 	PivotMMConfigs.MotionMagicExpo_kA = intakePivotMMKa.getNumber();
		// 	m_PivotMotor.getConfigurator().apply(PivotMMConfigs);
		// }

		// if (intakeMotorKp.getNumber() != IntakeMotorPIDConfig.kP
		// 		|| intakeMotorKi.getNumber() != IntakeMotorPIDConfig.kI
		// 		|| intakeMotorKd.getNumber() != IntakeMotorPIDConfig.kD
		// 		|| intakeMotorKs.getNumber() != IntakeMotorPIDConfig.kS) {
		// 	IntakeMotorPIDConfig.kP = intakeMotorKp.getNumber();
		// 	IntakeMotorPIDConfig.kI = intakeMotorKi.getNumber();
		// 	IntakeMotorPIDConfig.kD = intakeMotorKd.getNumber();
		// 	IntakeMotorPIDConfig.kS = intakeMotorKs.getNumber();

		// 	m_IntakeMotor.getConfigurator().apply(IntakeMotorPIDConfig);
		// }

		// if (m_IntakeState == IntakeStates.STOWED && !RobotStates.turretIsStowed.getAsBoolean()) {
		// 	// panic!
		// 	System.out.println(
		// 			"Attempting to stow intake before turret is stowed... reverting to DEFAULT");
		// 	m_IntakeState = IntakeStates.DEFAULT;
		// }

		if (m_IntakeState.intakeSpeed.abs(RotationsPerSecond) > 0) {
			m_IntakeMotor.setControl(m_IntakeRequest.withVelocity(m_IntakeState.intakeSpeed));
		} else {
			m_IntakeMotor.setControl(new CoastOut());
		}

		m_PivotMotor.setControl(m_PivotRequest.withPosition(m_IntakeState.pivotPosition));

	}
}