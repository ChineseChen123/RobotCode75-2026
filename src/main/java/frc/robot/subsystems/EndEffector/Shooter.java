// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.EndEffectorConstants.Shooter.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.RobotStates;

public class Shooter extends SubsystemBase {
	/** Creates a new Shooter. */
	public enum ShooterStates {
		DEFAULT(defaultFlywheelSpeed, defaultRollerSpeed),
		SHOOTING(null, null),
		REVERSING(reverseFlywheelSpeed, reverseRollerSpeed);

		AngularVelocity flywheelSpeed;
		AngularVelocity rollerSpeed;

		private ShooterStates(AngularVelocity flywheel, AngularVelocity roller) {
			this.flywheelSpeed = flywheel;
			this.rollerSpeed = roller;
		}
	}

	private ShooterStates m_ShooterState;
	private final TalonFX m_FlywheelMotor;
	private final TalonFX m_RollerMotor;
	private final VelocityTorqueCurrentFOC m_VelocityRequest = new VelocityTorqueCurrentFOC(0);

	public Shooter() {
		m_FlywheelMotor = new TalonFX(flywheelCanID, superstructureCANBusName);
		m_RollerMotor = new TalonFX(rollerCanID, superstructureCANBusName);

		m_FlywheelMotor.getConfigurator().apply(MotorConfigs.getFlywheelMotorConfiguration());
		m_RollerMotor.getConfigurator().apply(MotorConfigs.getRollerMotorConfiguration());

		m_ShooterState = ShooterStates.DEFAULT;
	}

	public double getFlyWheelVelocity() {
		return m_FlywheelMotor.getVelocity(true).getValue().in(RotationsPerSecond);
	}

	public double getRollerVelocity() {
		return m_RollerMotor.getVelocity(true).getValue().in(RotationsPerSecond);
	}

	public ShooterStates getShooterState() {
		return m_ShooterState;
	}

	public void setState(ShooterStates state) {
		m_ShooterState = state;
	}

	public AngularVelocity[] calculateShooterSpeeds(Pose2d robotPose) {
		// lookup table stuff
		// 0 > flywheel, 1 > roller
		return null;
	}

	@Override
	public void periodic() {
		if (m_ShooterState == ShooterStates.SHOOTING) {
			// variable speeds
			AngularVelocity[] velocities = calculateShooterSpeeds(RobotStates.robotPose.get());
			m_FlywheelMotor.setControl(m_VelocityRequest.withVelocity(velocities[0]));
			m_RollerMotor.setControl(m_VelocityRequest.withVelocity(velocities[1]));
		} else {
			m_FlywheelMotor.setControl(m_VelocityRequest.withVelocity(m_ShooterState.flywheelSpeed));
			m_RollerMotor.setControl(m_VelocityRequest.withVelocity(m_ShooterState.rollerSpeed));
		}
	}
}
