// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.IntakeIndexConstants.IndexerConstants.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

	private final DigitalInput m_BeamBreak;

	private final TalonFX m_IndexerMotor;
	private final TalonFX m_HopperMotor;
	private final VelocityTorqueCurrentFOC m_VelocityRequest = new VelocityTorqueCurrentFOC(0);

	public Indexer() {
		m_IndexerMotor = new TalonFX(indexerMotorCanID, superstructureCANBusName);
		m_HopperMotor = new TalonFX(hopperMotorCanID, superstructureCANBusName);

		m_IndexerState = IndexerStates.DEFAULT;
		m_BeamBreak = new DigitalInput(beamBreakPort);

		m_IndexerMotor.getConfigurator().apply(MotorConfigs.getIndexerMotorConfig());
		m_HopperMotor.getConfigurator().apply(MotorConfigs.getIndexerMotorConfig());
	}

	public boolean hasFuel() {
		return !m_BeamBreak.get();
	}

	public double getIndexerVelocity() {
		return m_IndexerMotor.getVelocity(true).getValue().in(RotationsPerSecond);
	}

	public IndexerStates getIndexerState() {
		return m_IndexerState;
	}

	public void setState(IndexerStates state) {
		m_IndexerState = state;
	}

	@Override
	public void periodic() {
		m_IndexerMotor.setControl(m_VelocityRequest.withVelocity(m_IndexerState.indexerSpeed));
		m_HopperMotor.setControl(m_VelocityRequest.withVelocity(m_IndexerState.hopperSpeed));
	}
}
