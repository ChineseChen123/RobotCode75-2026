// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;
import static frc.robot.Constants.ShooterConstants.Shooter.*;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ShooterPhysics;
import frc.robot.state.RobotStates;

public class Shooter extends SubsystemBase {
	/** Creates a new Shooter. */
	public enum ShooterStates {
		DEFAULT(defaultShooterSpeed),
		SHOOTING(null),
		REVERSING(reverseShooterSpeed);

		AngularVelocity shooterSpeed;

		private ShooterStates(AngularVelocity shooterSpeed) {
			this.shooterSpeed = shooterSpeed;
		}
	}

	private ShooterStates m_ShooterState;

	private final TalonFX m_ShooterMotor1;
	private final TalonFX m_ShooterMotor2;
	private final VelocityTorqueCurrentFOC m_VelocityRequest = new VelocityTorqueCurrentFOC(0);

	public Shooter() {
		m_ShooterMotor1 = new TalonFX(shooterMotor1CanID, superstructureCANBusName);
		m_ShooterMotor2 = new TalonFX(shooterMotor2CanID, superstructureCANBusName);

		m_ShooterMotor1.getConfigurator().apply(MotorConfigs.getShooterMotor1MotorConfiguration());
		m_ShooterMotor2.getConfigurator().apply(MotorConfigs.getShooterMotor2MotorConfiguration());

		m_ShooterState = ShooterStates.DEFAULT;
	}

	public double getFlyWheelVelocity() {
		return m_ShooterMotor1.getVelocity(true).getValue().in(RotationsPerSecond);
	}

	public double getRollerVelocity() {
		return m_ShooterMotor2.getVelocity(true).getValue().in(RotationsPerSecond);
	}

	public ShooterStates getShooterState() {
		return m_ShooterState;
	}

	public void setState(ShooterStates state) {
		m_ShooterState = state;
	}

	@Override
	public void periodic() {
		if (m_ShooterState == ShooterStates.SHOOTING) {
			// variable speeds
			AngularVelocity velocity = ShooterPhysics.calculateShooterSpeed(RobotStates.robotPose.get());
			m_ShooterMotor1.setControl(m_VelocityRequest.withVelocity(velocity));
			m_ShooterMotor2.setControl(m_VelocityRequest.withVelocity(velocity));
		} else {
			m_ShooterMotor1.setControl(m_VelocityRequest.withVelocity(m_ShooterState.shooterSpeed));
			m_ShooterMotor2.setControl(m_VelocityRequest.withVelocity(m_ShooterState.shooterSpeed));
		}
	}
}
