// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;
import static frc.robot.Constants.ShooterConstants.Shooter.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;
import frc.lib.util.ShooterPhysics;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.Shooter.MotorConfigs;
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

	private final Slot0Configs shooterPIDConfigs = new Slot0Configs();
	private final TunableNumber shooterKp;
	private final TunableNumber shooterKi;
	private final TunableNumber shooterKd;
	private final TunableNumber shooterKs;

	public Shooter() {
		m_ShooterMotor1 = new TalonFX(shooterMotor1CanID, superstructureCANBusName);
		m_ShooterMotor2 = new TalonFX(shooterMotor2CanID, superstructureCANBusName);

		m_ShooterMotor1.getConfigurator().apply(MotorConfigs.getShooterMotor1MotorConfiguration());
		m_ShooterMotor2.getConfigurator().apply(MotorConfigs.getShooterMotor2MotorConfiguration());

		m_ShooterState = ShooterStates.DEFAULT;

		shooterPIDConfigs
				.withKP(MotorConfigs.shooterMotor1VelocityKP)
				.withKI(MotorConfigs.shooterMotor1VelocityKI)
				.withKD(MotorConfigs.shooterMotor1VelocityKD)
				.withKS(MotorConfigs.shooterMotor1VelocityKS);

		shooterKp = new TunableNumber("Shooter/Kp", MotorConfigs.shooterMotor1VelocityKP);
		shooterKi = new TunableNumber("Shooter/Ki", MotorConfigs.shooterMotor1VelocityKI);
		shooterKd = new TunableNumber("Shooter/Kd", MotorConfigs.shooterMotor1VelocityKD);
		shooterKs = new TunableNumber("Shooter/Ks", MotorConfigs.shooterMotor1VelocityKS);
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

		if (shooterKp.getNumber() != shooterPIDConfigs.kP
				|| shooterKi.getNumber() != shooterPIDConfigs.kI
				|| shooterKd.getNumber() != shooterPIDConfigs.kD
				|| shooterKs.getNumber() != shooterPIDConfigs.kS) {
			shooterPIDConfigs.kP = shooterKp.getNumber();
			shooterPIDConfigs.kI = shooterKi.getNumber();
			shooterPIDConfigs.kD = shooterKd.getNumber();
			shooterPIDConfigs.kS = shooterKs.getNumber();
			m_ShooterMotor1.getConfigurator().apply(shooterPIDConfigs);
			m_ShooterMotor2.getConfigurator().apply(shooterPIDConfigs);
		}

		if (m_ShooterState == ShooterStates.SHOOTING) {
					// lookup table stuff
		Pose2d targetHubPose =
				DriverStation.getAlliance().isPresent()
								&& DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
						? FieldConstants.blueHub
						: FieldConstants.redHub;

			// variable speeds
				if (ShooterConstants.useVirtualTarget) {
					targetHubPose =
							ShooterPhysics.getVirtualTarget(targetHubPose, RobotStates.fieldRelativeSpeeds.get(), 5);
				}
			AngularVelocity velocity =
					ShooterPhysics.calculateShooterSpeed(RobotStates.robotPose.get(), targetHubPose);
			// AngularVelocity velocity =
			// ShooterPhysics.calculateShooterSpeed(RobotStates.robotPose.get());

			m_ShooterMotor1.setControl(m_VelocityRequest.withVelocity(velocity));
			m_ShooterMotor2.setControl(m_VelocityRequest.withVelocity(velocity));
		} else {
			m_ShooterMotor1.setControl(m_VelocityRequest.withVelocity(m_ShooterState.shooterSpeed));
			m_ShooterMotor2.setControl(m_VelocityRequest.withVelocity(m_ShooterState.shooterSpeed));
		}
	}
}
