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
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;
import frc.robot.Constants.IntakeIndexConstants.IndexerConstants.MotorConfigs;

public class Indexer extends SubsystemBase {
	/** Creates a new Shooter. */
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

	private IndexerStates m_IndexerState;

	// private final DigitalInput m_BeamBreak;

	private final TalonFX m_IndexerMotor;
	private final TalonFX m_HopperMotor;
	private final VelocityDutyCycle m_IndexerDutyCycle = new VelocityDutyCycle(0);
	private final VelocityTorqueCurrentFOC m_IndexerTorqueCurrent = new VelocityTorqueCurrentFOC(0);
	private final VelocityTorqueCurrentFOC m_HopperRequest = new VelocityTorqueCurrentFOC(0);

	private final Slot0Configs indexerConfigs = new Slot0Configs();
	// private final TunableNumber indexerKp =
	// 		new TunableNumber("Indexer/Kp", MotorConfigs.indexerVelocityKP);
	// private final TunableNumber indexerKd =
	// 		new TunableNumber("Indexer/Kd", MotorConfigs.indexerVelocityKD);
	// private final TunableNumber indexerKs =
	// 		new TunableNumber("Indexer/Ks", MotorConfigs.indexerVelocityKS);
	// private final TunableNumber indexerKv =
	// 		new TunableNumber("Indexer/Kv", MotorConfigs.indexerVelocityKV);

	private final Slot0Configs hopperConfigs = new Slot0Configs();

	// private final TunableNumber hopperKp =
	// 		new TunableNumber("Hopper/Kp", MotorConfigs.hopperVelocityKP);
	// private final TunableNumber hopperKd =
	// 		new TunableNumber("Hopper/Kd", MotorConfigs.hopperVelocityKD);
	// private final TunableNumber hopperKs =
	// 		new TunableNumber("Hopper/Ks", MotorConfigs.hopperVelocityKS);

	// private final TunableNumber indexerSpeed =
	// 		new TunableNumber("Indexer/IndexerSpeed", defaultIndexerSpeed.in(RotationsPerSecond));
	// private final TunableNumber hopperSpeed =
	// 		new TunableNumber("Hopper/HopperSpeed", defaultHopperSpeed.in(RotationsPerSecond));

	public Indexer() {
		m_IndexerMotor = new TalonFX(indexerMotorCanID, superstructureCANBusName);
		m_HopperMotor = new TalonFX(hopperMotorCanID, superstructureCANBusName);

		m_IndexerState = IndexerStates.DEFAULT;
		// m_BeamBreak = new DigitalInput(beamBreakPort);

		m_IndexerMotor.getConfigurator().apply(MotorConfigs.getIndexerMotorConfig());
		m_HopperMotor.getConfigurator().apply(MotorConfigs.getHopperMotorConfig());

		m_IndexerTorqueCurrent.UpdateFreqHz = 0;
		m_IndexerTorqueCurrent.UseTimesync = true;
		m_HopperRequest.UpdateFreqHz = 0;
		m_HopperRequest.UseTimesync = true;

		indexerConfigs
				.withKP(MotorConfigs.indexerVelocityKP)
				.withKD(MotorConfigs.indexerVelocityKD)
				.withKS(MotorConfigs.indexerVelocityKS)
				.withKV(MotorConfigs.indexerVelocityKV);

		hopperConfigs
				.withKP(MotorConfigs.hopperVelocityKP)
				.withKD(MotorConfigs.hopperVelocityKD)
				.withKS(MotorConfigs.hopperVelocityKS);
	}

	public boolean hasFuel() {
		// return !m_BeamBreak.get();
		return false;
	}

	@Logged(key = "Indexer Velocity", importance = Importance.DEBUG)
	public double getIndexerVelocityRPS() {
		return m_IndexerMotor.getVelocity(true).getValue().in(RotationsPerSecond);
	}

	// @Logged(key = "Hopper Velocity", importance = Importance.DEBUG)
	public double getHopperVelocityRPS() {
		return m_HopperMotor.getVelocity(true).getValue().in(RotationsPerSecond);
	}

	@Logged(key = "Indexer Current", importance = Importance.DEBUG)
	public double getIndexerCurrent() {
		return m_IndexerMotor.getStatorCurrent(true).getValue().in(Amps);
	}

	// @Logged(key = "Indexer Supply Current", importance = Importance.DEBUG)
	public double getIndexerSupplyCurrent() {
		return m_IndexerMotor.getSupplyCurrent(true).getValue().in(Amps);
	}

	public IndexerStates getIndexerState() {
		return m_IndexerState;
	}

	public void setState(IndexerStates state) {
		m_IndexerState = state;
	}

	@Override
	public void periodic() {

		// if (indexerKp.getNumber() != indexerConfigs.kP
		// 		|| indexerKd.getNumber() != indexerConfigs.kD
		// 		|| indexerKs.getNumber() != indexerConfigs.kS
		// 		|| indexerKv.getNumber() != indexerConfigs.kV) {
		// 	indexerConfigs
		// 			.withKP(indexerKp.getNumber())
		// 			.withKD(indexerKd.getNumber())
		// 			.withKS(indexerKs.getNumber())
		// 			.withKV(indexerKv.getNumber());
		// 	m_IndexerMotor.getConfigurator().apply(indexerConfigs);
		// }

		// if (hopperKp.getNumber() != hopperConfigs.kP
		// 		|| hopperKd.getNumber() != hopperConfigs.kD
		// 		|| hopperKs.getNumber() != hopperConfigs.kS) {
		// 	hopperConfigs
		// 			.withKP(hopperKp.getNumber())
		// 			.withKD(hopperKd.getNumber())
		// 			.withKS(hopperKs.getNumber());
		// 	m_HopperMotor.getConfigurator().apply(hopperConfigs);
		// }

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
	}
}
