// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.IntakeIndexConstants.IndexerConstants.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
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
import frc.robot.state.RobotStates;

public class Indexer extends SubsystemBase {

	public enum IndexerStates {
		DEFAULT(defaultIndexerSpeed),
		SHOOTING(shootingIndexerSpeed),
		REVERSING(reverseIndexerSpeed);

		public final AngularVelocity indexerSpeed;

		private IndexerStates(AngularVelocity indexerSpeed) {
			this.indexerSpeed = indexerSpeed;
		}
	}

	private final TalonFX m_IndexerMotor;
	private final VelocityTorqueCurrentFOC m_IndexerTorqueCurrent = new VelocityTorqueCurrentFOC(0);

	private final Slot0Configs indexerConfigs = new Slot0Configs();
	private TunableNumber indexerKp;
	private TunableNumber indexerKd;
	private TunableNumber indexerKs;
	private TunableNumber indexerKv;

	private IndexerStates m_IndexerState;
	private AngularVelocity currentIndexerVelocity = RotationsPerSecond.of(0);

	/** Creates a new Indexer. */
	public Indexer() {
		m_IndexerMotor = new TalonFX(indexerMotorCanID, superstructureCANBusName);
		m_IndexerState = IndexerStates.DEFAULT;

		m_IndexerMotor.getConfigurator().apply(MotorConfigs.getIndexerBangBangConfiguration());

		m_IndexerTorqueCurrent.UpdateFreqHz = 50;

		// indexerConfigs
		// 		.withKP(MotorConfigs.indexerVelocityKP)
		// 		.withKD(MotorConfigs.indexerVelocityKD)
		// 		.withKS(MotorConfigs.indexerVelocityKS)
		// 		.withKV(MotorConfigs.indexerVelocityKV);

		// initTunables();
	}

	@Logged(key = "Indexer Velocity", importance = Importance.DEBUG)
	public double getIndexerVelocityRPS() {
		return currentIndexerVelocity.in(RotationsPerSecond);
	}

	public AngularVelocity getIndexerVelocity() {
		return m_IndexerMotor.getVelocity(true).getValue();
	}

	@Logged(key = "Indexer Current", importance = Importance.DEBUG)
	public double getIndexerCurrent() {
		return m_IndexerMotor.getStatorCurrent(true).getValue().in(Amps);
	}

	public boolean isIndexerUpToSpeed() {
		return (currentIndexerVelocity.minus(m_IndexerState.indexerSpeed).abs(RotationsPerSecond)
				< indexerSpeedThresholdRPS);
	}

	public boolean isIndexerRunning() {
		return (currentIndexerVelocity.abs(RotationsPerSecond) < 0.01);
	}

	public IndexerStates getIndexerState() {
		return m_IndexerState;
	}

	public void setState(IndexerStates state) {
		m_IndexerState = state;
	}

	public Command setStateCommand(IndexerStates state) {
		return new InstantCommand(() -> setState(state), this)
				.repeatedly()
				.finallyDo(() -> setState(IndexerStates.DEFAULT));
	}

	public Command setStateCommandPersistent(IndexerStates state) {
		return new InstantCommand(() -> setState(state), this);
	}

	public void updateCache() {
		currentIndexerVelocity = m_IndexerMotor.getVelocity(true).getValue();
	}

	@Override
	public void periodic() {
		updateCache();
		// updateTunables();
		

		boolean indexerJammed = RobotStates.indexerJammed.getAsBoolean();

		if (m_IndexerState.indexerSpeed.baseUnitMagnitude() == 0) {
			m_IndexerMotor.setControl(new CoastOut());
		} else {
			m_IndexerMotor.setControl(m_IndexerTorqueCurrent.withVelocity(indexerJammed ? RotationsPerSecond.of(-15) : m_IndexerState.indexerSpeed));
		}
	}

	// public void initTunables() {
	// 	if (RobotConstants.TuningModes.tuneIndexer) {
	// 		indexerKp = new TunableNumber("Indexer/Kp", MotorConfigs.indexerVelocityKP);
	// 		indexerKd = new TunableNumber("Indexer/Kd", MotorConfigs.indexerVelocityKD);
	// 		indexerKs = new TunableNumber("Indexer/Ks", MotorConfigs.indexerVelocityKS);
	// 		indexerKv = new TunableNumber("Indexer/Kv", MotorConfigs.indexerVelocityKV);
	// 	}
	// }

	// public void updateTunables() {
	// 	if (RobotConstants.TuningModes.tuneIndexer
	// 			&& (indexerKp.getNumber() != indexerConfigs.kP
	// 					|| indexerKd.getNumber() != indexerConfigs.kD
	// 					|| indexerKs.getNumber() != indexerConfigs.kS
	// 					|| indexerKv.getNumber() != indexerConfigs.kV)) {
	// 		indexerConfigs
	// 				.withKP(indexerKp.getNumber())
	// 				.withKD(indexerKd.getNumber())
	// 				.withKS(indexerKs.getNumber())
	// 				.withKV(indexerKv.getNumber());
	// 		m_IndexerMotor.getConfigurator().apply(indexerConfigs);
	// 	}
	// }
}
