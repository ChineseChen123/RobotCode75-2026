// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffectors;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;
import static frc.robot.Constants.EndEffectorConstants.*;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants.MotorConfigs;
import frc.robot.state.RobotStates;

public class Indexer extends SubsystemBase {
	/** Creates a new Shooter. */
	public enum IndexerStates {
		DEFAULT(defaultIndexerSpeed),
		RUNNING(runningIndexerSpeed),
		REVERSING(reverseIndexerSpeed);

		AngularVelocity indexerSpeed;

		private IndexerStates(AngularVelocity indexerSpeed) {
			this.indexerSpeed = indexerSpeed;
		}
	}

	private IndexerStates m_IndexerState;

 	private DigitalInput m_BeamBreak;

	private final TalonFX m_IndexerMotor;
	private final VelocityTorqueCurrentFOC m_VelocityRequest = new VelocityTorqueCurrentFOC(0);

	public Indexer() {
		m_IndexerMotor = new TalonFX(indexerMotorCanID, superstructureCANBusName);

    	m_IndexerState = IndexerStates.DEFAULT;
    	m_BeamBreak = new DigitalInput(beamBreakPort);

		m_IndexerMotor.getConfigurator().apply(MotorConfigs.getIndexerMotorConfig());

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
  }
}