// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeIndexConstants.IndexerConstants.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.dashboard.TunableNumber;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;
import frc.robot.Constants.IntakeIndexConstants.IndexerConstants.MotorConfigs;

public class Indexer extends SubsystemBase {
	/** Creates a new Shooter. */
	public enum IndexerStates {
		DEFAULT(defaultIndexerSpeed, defaultHopperSpeed),
		INDEXING(runningIndexerSpeed, runningHopperSpeed),
		READYTOSHOOT(defaultIndexerSpeed, runningHopperSpeed),
		SHOOTING(shootingIndexerSpeed, runningHopperSpeed),
		REVERSING(reverseIndexerSpeed, reverseHopperSpeed);

		AngularVelocity indexerSpeed;
		AngularVelocity hopperSpeed;

		private IndexerStates(AngularVelocity indexerSpeed, AngularVelocity hopperSpeed) {
			this.indexerSpeed = indexerSpeed;
			this.hopperSpeed = hopperSpeed;
		}
	}

	private IndexerStates m_IndexerState;

	// private final DigitalInput m_BeamBreak;

	private final TalonFX m_IndexerMotor;
	private final TalonFX m_HopperMotor;
	private final VelocityTorqueCurrentFOC m_IndexerRequest = new VelocityTorqueCurrentFOC(0);
	private final VelocityTorqueCurrentFOC m_HopperRequest = new VelocityTorqueCurrentFOC(0);

	private final VoltageOut m_sysIdRequest = new VoltageOut(0.0);

	private final Slot0Configs indexerConfigs = new Slot0Configs();
	private final TunableNumber indexerKp =
			new TunableNumber("Indexer/Kp", MotorConfigs.indexerVelocityKP);
	private final TunableNumber indexerKd =
			new TunableNumber("Indexer/Kd", MotorConfigs.indexerVelocityKD);
	private final TunableNumber indexerKs =
			new TunableNumber("Indexer/Ks", MotorConfigs.indexerVelocityKS);

	private final Slot0Configs hopperConfigs = new Slot0Configs();
	private final TunableNumber hopperKp =
			new TunableNumber("Hopper/Kp", MotorConfigs.hopperVelocityKP);
	private final TunableNumber hopperKd =
			new TunableNumber("Hopper/Kd", MotorConfigs.hopperVelocityKD);
	private final TunableNumber hopperKs =
			new TunableNumber("Hopper/Ks", MotorConfigs.hopperVelocityKS);

	private final TunableNumber indexerSpeed =
			new TunableNumber("Indexer/IndexerSpeed", defaultIndexerSpeed.in(RotationsPerSecond));
	private final TunableNumber hopperSpeed =
			new TunableNumber("Hopper/HopperSpeed", defaultHopperSpeed.in(RotationsPerSecond));

	private final SysIdRoutine m_sysIdRoutine;

	public Indexer() {
		m_IndexerMotor = new TalonFX(indexerMotorCanID, superstructureCANBusName);
		m_HopperMotor = new TalonFX(hopperMotorCanID, superstructureCANBusName);

		m_IndexerState = IndexerStates.DEFAULT;
		// m_BeamBreak = new DigitalInput(beamBreakPort);

		m_IndexerMotor.getConfigurator().apply(MotorConfigs.getIndexerMotorConfig());
		m_HopperMotor.getConfigurator().apply(MotorConfigs.getHopperMotorConfig());

		m_IndexerRequest.UpdateFreqHz = 0;
		m_IndexerRequest.UseTimesync = true;
		m_HopperRequest.UpdateFreqHz = 0;
		m_HopperRequest.UseTimesync = true;

		indexerConfigs
				.withKP(MotorConfigs.indexerVelocityKP)
				.withKD(MotorConfigs.indexerVelocityKD)
				.withKS(MotorConfigs.indexerVelocityKS);

		hopperConfigs
				.withKP(MotorConfigs.hopperVelocityKP)
				.withKD(MotorConfigs.hopperVelocityKD)
				.withKS(MotorConfigs.hopperVelocityKS);

		m_sysIdRoutine =
				new SysIdRoutine(
						new SysIdRoutine.Config(
								null, // Use default ramp rate (1 V/s)
								Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
								null, // Use default timeout (10 s)
								// Log state with Phoenix SignalLogger class
								(state) -> SignalLogger.writeString("state", state.toString())),
						new SysIdRoutine.Mechanism(
								(volts) -> {
									m_IndexerMotor.setControl(m_sysIdRequest.withOutput(volts.in(Volts)));
								},
								null,
								this));
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return m_sysIdRoutine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return m_sysIdRoutine.dynamic(direction);
	}

	public boolean hasFuel() {
		// return !m_BeamBreak.get();
		return false;
	}

	@Logged(key = "Indexer Velocity", importance = Importance.DEBUG)
	public double getIndexerVelocityRPS() {
		return m_IndexerMotor.getVelocity(true).getValue().in(RotationsPerSecond);
	}

	@Logged(key = "Hopper Velocity", importance = Importance.DEBUG)
	public double getHopperVelocityRPS() {
		return m_HopperMotor.getVelocity(true).getValue().in(RotationsPerSecond);
	}

	public IndexerStates getIndexerState() {
		return m_IndexerState;
	}

	public void setState(IndexerStates state) {
		m_IndexerState = state;
	}

	@Override
	public void periodic() {

		if (indexerKp.getNumber() != indexerConfigs.kP
				|| indexerKd.getNumber() != indexerConfigs.kD
				|| indexerKs.getNumber() != indexerConfigs.kS) {
			indexerConfigs
					.withKP(indexerKp.getNumber())
					.withKD(indexerKd.getNumber())
					.withKS(indexerKs.getNumber());
			m_IndexerMotor.getConfigurator().apply(indexerConfigs);
		}

		if (hopperKp.getNumber() != hopperConfigs.kP
				|| hopperKd.getNumber() != hopperConfigs.kD
				|| hopperKs.getNumber() != hopperConfigs.kS) {
			hopperConfigs
					.withKP(hopperKp.getNumber())
					.withKD(hopperKd.getNumber())
					.withKS(hopperKs.getNumber());
			m_HopperMotor.getConfigurator().apply(hopperConfigs);
		}

		if (m_IndexerState.indexerSpeed.baseUnitMagnitude() == 0) {
			m_IndexerMotor.setControl(new CoastOut());
		} else {
			m_IndexerMotor.setControl(m_IndexerRequest.withVelocity(m_IndexerState.indexerSpeed));
		}

		if (m_IndexerState.hopperSpeed.baseUnitMagnitude() == 0) {
			m_HopperMotor.setControl(new CoastOut());
		} else {
			m_HopperMotor.setControl(m_HopperRequest.withVelocity(m_IndexerState.hopperSpeed));
		}
	}
}
