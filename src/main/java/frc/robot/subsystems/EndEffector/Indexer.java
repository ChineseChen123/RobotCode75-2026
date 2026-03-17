// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.IntakeIndexConstants.IndexerConstants.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;
import frc.robot.Constants.IntakeIndexConstants.IndexerConstants.MotorConfigs;
import frc.robot.Constants.RobotConstants;

public class Indexer extends SubsystemBase {

	// ── State enum ───────────────────────────────────────────────────────────────

	public enum IndexerStates {
		DEFAULT(defaultIndexerSpeed, defaultHopperSpeed, MotorConfigs.getIndexerMotorConfig()),
		READYTOSHOOT(defaultIndexerSpeed, runningHopperSpeed, MotorConfigs.getIndexerMotorConfig()),
		SHOOTING(shootingIndexerSpeed, runningHopperSpeed, MotorConfigs.getIndexerMotorConfig()),
		REVERSING(reverseIndexerSpeed, reverseHopperSpeed, MotorConfigs.getIndexerMotorConfig());

		AngularVelocity indexerSpeed;
		AngularVelocity hopperSpeed;
		TalonFXConfiguration indexerConfig;

		private IndexerStates(
				AngularVelocity indexerSpeed,
				AngularVelocity hopperSpeed,
				TalonFXConfiguration indexerConfig) {
			this.indexerSpeed = indexerSpeed;
			this.hopperSpeed = hopperSpeed;
			this.indexerConfig = indexerConfig;
		}
	}

	// ── Hardware ─────────────────────────────────────────────────────────────────

	private final TalonFX m_IndexerMotor;
	private final TalonFX m_HopperMotor;
	private final TalonFX m_ParallelMotor;

	// ── Control requests / configs ───────────────────────────────────────────────

	private final VelocityDutyCycle m_IndexerDutyCycle = new VelocityDutyCycle(0);
	private final VelocityTorqueCurrentFOC m_IndexerTorqueCurrent = new VelocityTorqueCurrentFOC(0);
	private final VelocityTorqueCurrentFOC m_HopperRequest = new VelocityTorqueCurrentFOC(0);
	private final VelocityTorqueCurrentFOC m_ParallelRollerRequest = new VelocityTorqueCurrentFOC(0);

	private final Slot0Configs indexerConfigs = new Slot0Configs();
	private TunableNumber indexerKp;
	private TunableNumber indexerKd;
	private TunableNumber indexerKs;
	private TunableNumber indexerKv;
	private TunableNumber indexerSpeed;

	private final Slot0Configs hopperConfigs = new Slot0Configs();

	private TunableNumber hopperKp;
	private TunableNumber hopperKd;
	private TunableNumber hopperKs;

	private final Slot0Configs parallelConfigs = new Slot0Configs();

	private TunableNumber parallelKp;
	private TunableNumber parallelKd;
	private TunableNumber parallelKs;
	private TunableNumber parallelKv;
	private TunableNumber parallelSpeed;

	// ── Internal state ───────────────────────────────────────────────────────────

	private IndexerStates m_IndexerState;

	private boolean runParallel = false;

	private AngularVelocity currentIndexerVelocity = RotationsPerSecond.of(0);
	private AngularVelocity currentHopperVelocity = RotationsPerSecond.of(0);
	private AngularVelocity currentParallelVelocity = RotationsPerSecond.of(0);

	/** Creates a new Indexer. */
	public Indexer() {
		m_IndexerMotor = new TalonFX(indexerMotorCanID, superstructureCANBusName);
		m_HopperMotor = new TalonFX(hopperMotorCanID, superstructureCANBusName);
		m_ParallelMotor = new TalonFX(parallelMotorCanID, superstructureCANBusName);

		m_IndexerState = IndexerStates.DEFAULT;

		m_IndexerMotor.getConfigurator().apply(MotorConfigs.getIndexerMotorConfig());
		m_HopperMotor.getConfigurator().apply(MotorConfigs.getHopperMotorConfig());
		m_ParallelMotor.getConfigurator().apply(MotorConfigs.getParallelMotorConfig());

		m_IndexerTorqueCurrent.UpdateFreqHz = 0;
		m_IndexerTorqueCurrent.UseTimesync = true;
		m_HopperRequest.UpdateFreqHz = 0;
		m_HopperRequest.UseTimesync = true;
		m_ParallelRollerRequest.UpdateFreqHz = 0;
		m_ParallelRollerRequest.UseTimesync = true;

		indexerConfigs
				.withKP(MotorConfigs.indexerVelocityKP)
				.withKD(MotorConfigs.indexerVelocityKD)
				.withKS(MotorConfigs.indexerVelocityKS)
				.withKV(MotorConfigs.indexerVelocityKV);

		hopperConfigs
				.withKP(MotorConfigs.hopperVelocityKP)
				.withKD(MotorConfigs.hopperVelocityKD)
				.withKS(MotorConfigs.hopperVelocityKS);

		parallelConfigs
				.withKP(MotorConfigs.parallelVelocityKP)
				.withKD(MotorConfigs.parallelVelocityKD)
				.withKS(MotorConfigs.parallelVelocityKS)
				.withKV(MotorConfigs.parallelVelocityKV);

		initTunables();
	}

	// ── Sensor / state accessors ─────────────────────────────────────────────────

	@Logged(key = "Indexer Velocity", importance = Importance.DEBUG)
	public double getIndexerVelocityRPS() {
		return currentIndexerVelocity.in(RotationsPerSecond);
	}

	public double getHopperVelocityRPS() {
		return currentHopperVelocity.in(RotationsPerSecond);
	}

	@Logged(key = "Parallel Roller Velocity", importance = Importance.DEBUG)
	public double getParallelVelocityRPS() {
		return currentParallelVelocity.in(RotationsPerSecond);
	}

	@Logged(key = "Indexer Current", importance = Importance.DEBUG)
	public double getIndexerCurrent() {
		return m_IndexerMotor.getStatorCurrent(true).getValue().in(Amps);
	}

	public IndexerStates getIndexerState() {
		return m_IndexerState;
	}

	public void setState(IndexerStates state) {
		m_IndexerState = state;
	}

	// ── Commands ─────────────────────────────────────────────────────────────────

	public Command setStateCommand(IndexerStates state) {
		return new InstantCommand(() -> setState(state), this)
				.repeatedly()
				.finallyDo(() -> setState(IndexerStates.DEFAULT));
	}

	public Command setStateCommandPersistent(IndexerStates state) {
		return new InstantCommand(() -> setState(state), this);
	}

	// ── Updates ──────────────────────────────────────────────────────────────────

	public void updateCache() {
		currentIndexerVelocity = m_IndexerMotor.getVelocity(true).getValue();
		currentHopperVelocity = m_HopperMotor.getVelocity(true).getValue();
		currentParallelVelocity = m_ParallelMotor.getVelocity(true).getValue();
	}

	// ── WPILib lifecycle ─────────────────────────────────────────────────────────

	@Override
	public void periodic() {

		updateCache();
		updateTunables();

		if (m_IndexerState.indexerSpeed.baseUnitMagnitude() == 0) {
			m_IndexerMotor.setControl(new CoastOut());
		} else {
			m_IndexerMotor.setControl(m_IndexerTorqueCurrent.withVelocity(m_IndexerState.indexerSpeed));
		}

		if (m_IndexerState.hopperSpeed.baseUnitMagnitude() == 0) {
			m_HopperMotor.setControl(new CoastOut());
		} else {
			m_HopperMotor.setControl(m_HopperRequest.withVelocity(m_IndexerState.hopperSpeed));
		}

		runParallel =
				runParallel
						? !(currentIndexerVelocity.abs(RotationsPerSecond) < 0.01)
						: (currentIndexerVelocity.minus(m_IndexerState.indexerSpeed).abs(RotationsPerSecond)
								< indexerSpeedThresholdRPS);
		if (m_IndexerState == IndexerStates.SHOOTING && runParallel) {
			m_ParallelMotor.setControl(m_ParallelRollerRequest.withVelocity(runningParallelSpeed));
		} else if (m_IndexerState == IndexerStates.DEFAULT) {
			m_ParallelMotor.setControl(m_ParallelRollerRequest.withVelocity(reverseParallelSpeed));
		} else {
			m_ParallelMotor.setControl(new StaticBrake());
		}
	}

	// ── Tuning ───────────────────────────────────────────────────────────────────

	public void initTunables() {
		if (RobotConstants.TuningModes.tuneIndexer) {
			indexerKp = new TunableNumber("Indexer/Kp", MotorConfigs.indexerVelocityKP);
			indexerKd = new TunableNumber("Indexer/Kd", MotorConfigs.indexerVelocityKD);
			indexerKs = new TunableNumber("Indexer/Ks", MotorConfigs.indexerVelocityKS);
			indexerKv = new TunableNumber("Indexer/Kv", MotorConfigs.indexerVelocityKV);
			indexerSpeed = new TunableNumber("Indexer/Speed", defaultIndexerSpeed.in(RotationsPerSecond));
		}

		if (RobotConstants.TuningModes.tuneHopper) {
			hopperKp = new TunableNumber("Hopper/Kp", MotorConfigs.hopperVelocityKP);
			hopperKd = new TunableNumber("Hopper/Kd", MotorConfigs.hopperVelocityKD);
			hopperKs = new TunableNumber("Hopper/Ks", MotorConfigs.hopperVelocityKS);
		}

		if (RobotConstants.TuningModes.tuneParallel) {
			parallelKp = new TunableNumber("Parallel/Kp", MotorConfigs.parallelVelocityKP);
			parallelKd = new TunableNumber("Parallel/Kd", MotorConfigs.parallelVelocityKD);
			parallelKs = new TunableNumber("Parallel/Ks", MotorConfigs.parallelVelocityKS);
			parallelKv = new TunableNumber("Parallel/Kv", MotorConfigs.parallelVelocityKV);
			parallelSpeed =
					new TunableNumber("Parallel/Speed", runningParallelSpeed.in(RotationsPerSecond));
		}
	}

	public void updateTunables() {
		if (RobotConstants.TuningModes.tuneIndexer
				&& (indexerKp.getNumber() != indexerConfigs.kP
						|| indexerKd.getNumber() != indexerConfigs.kD
						|| indexerKs.getNumber() != indexerConfigs.kS
						|| indexerKv.getNumber() != indexerConfigs.kV)) {
			indexerConfigs
					.withKP(indexerKp.getNumber())
					.withKD(indexerKd.getNumber())
					.withKS(indexerKs.getNumber())
					.withKV(indexerKv.getNumber());
			m_IndexerMotor.getConfigurator().apply(indexerConfigs);
		}

		if (RobotConstants.TuningModes.tuneHopper
				&& (hopperKp.getNumber() != hopperConfigs.kP
						|| hopperKd.getNumber() != hopperConfigs.kD
						|| hopperKs.getNumber() != hopperConfigs.kS)) {
			hopperConfigs
					.withKP(hopperKp.getNumber())
					.withKD(hopperKd.getNumber())
					.withKS(hopperKs.getNumber());
			m_HopperMotor.getConfigurator().apply(hopperConfigs);
		}

		if (RobotConstants.TuningModes.tuneParallel
				&& (parallelKp.getNumber() != parallelConfigs.kP
						|| parallelKd.getNumber() != parallelConfigs.kD
						|| parallelKs.getNumber() != parallelConfigs.kS
						|| parallelKv.getNumber() != parallelConfigs.kV)) {
			parallelConfigs
					.withKP(parallelKp.getNumber())
					.withKD(parallelKd.getNumber())
					.withKS(parallelKs.getNumber())
					.withKV(parallelKv.getNumber());
			m_ParallelMotor.getConfigurator().apply(parallelConfigs);
		}
	}
}
