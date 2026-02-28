// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;
import static frc.robot.Constants.ShooterTurretConstants.ShooterConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.dashboard.TunableNumber;
import frc.lib.util.PeddieBounds;
import frc.lib.util.ShooterPhysics;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;
import frc.robot.Constants.ShooterTurretConstants;
import frc.robot.Constants.ShooterTurretConstants.ShooterConstants.MotorConfigs;
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
	private AngularVelocity shooterTargetVelocity = RotationsPerSecond.of(0);

	private final TalonFX m_ShooterMotor1;
	private final TalonFX m_ShooterMotor2;
	private final VelocityTorqueCurrentFOC m_VelocityRequest;
	private final Follower m_FollowerRequest;

	private final Slot0Configs shooterPIDConfigs = new Slot0Configs();
	private final TunableNumber shooterKp;
	private final TunableNumber shooterKd;
	private final TunableNumber shooterKv;

	private final TunableNumber targetSpeed;

	public Shooter() {
		m_ShooterMotor1 = new TalonFX(shooterMotor1CanID, superstructureCANBusName);
		m_ShooterMotor2 = new TalonFX(shooterMotor2CanID, superstructureCANBusName);

		m_ShooterMotor1.getConfigurator().apply(MotorConfigs.getShooterMotorConfiguration());
		m_ShooterMotor2.getConfigurator().apply(MotorConfigs.getShooterMotorConfiguration());

		m_VelocityRequest = new VelocityTorqueCurrentFOC(RotationsPerSecond.of(0));
		m_FollowerRequest = new Follower(m_ShooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed);

		m_ShooterState = ShooterStates.DEFAULT;

		shooterPIDConfigs
				.withKP(MotorConfigs.shooterMotorVelocityKP)
				.withKI(MotorConfigs.shooterMotorVelocityKI)
				.withKD(MotorConfigs.shooterMotorVelocityKD)
				.withKS(MotorConfigs.shooterMotorVelocityKS)
				.withKV(MotorConfigs.shooterMotorVelocityKV)
				.withKA(MotorConfigs.shooterMotorVelocityKA);

		shooterKp = new TunableNumber("Shooter/Kp", MotorConfigs.shooterMotorVelocityKP);
		shooterKd = new TunableNumber("Shooter/Kd", MotorConfigs.shooterMotorVelocityKD);
		shooterKv = new TunableNumber("Shooter/Kv", MotorConfigs.shooterMotorVelocityKV);
		targetSpeed = new TunableNumber("Shooter/Target Speed RPM", 0);

		m_VelocityRequest.UpdateFreqHz = 0;
		m_VelocityRequest.UseTimesync = true;
	}

	@Logged(key = "Shooter Motor Velocity RPM", importance = Importance.CRITICAL)
	public double getMotorVelocityRPM() {
		return getVelocity().in(RotationsPerSecond) * 60;
	}

	@Logged(key = "Shooter Wheel Velocity RPM", importance = Importance.CRITICAL)
	public double getWheelVelocityRPM() {
		return getMotorVelocityRPM() * shooterGearRatio;
	}

	public AngularVelocity getVelocity() {
		return m_ShooterMotor1
				.getVelocity(true)
				.getValue()
				.plus(m_ShooterMotor2.getVelocity(true).getValue())
				.div(2);
	}

	@Logged(key = "Shooter Target Velocity RPM", importance = Importance.CRITICAL)
	public double targetVelocityRPM() {
		return shooterTargetVelocity.in(RotationsPerSecond) * 60;
	}

	public boolean atTargetVelocity() {
		return getVelocity().minus(shooterTargetVelocity).abs(RotationsPerSecond)
				< shooterVelocityTolerance;
	}

	public ShooterStates getShooterState() {
		return m_ShooterState;
	}

	public void setState(ShooterStates state) {
		m_ShooterState = state;
	}

	@Override
	public void periodic() {

		// if (m_ShooterState == ShooterStates.SHOOTING) {
		// 	m_ShooterMotor1.setControl(
		// 			m_VelocityRequest.withVelocity(RotationsPerSecond.of(targetSpeed.getNumber() / 60)));
		// 	m_ShooterMotor2.setControl(m_FollowerRequest);
		// 	shooterTargetVelocity = RotationsPerSecond.of(targetSpeed.getNumber() / 60);
		// } else {
		// 	m_ShooterMotor1.setControl(new CoastOut());
		// 	m_ShooterMotor2.setControl(m_FollowerRequest);
		// }

		if (shooterKp.getNumber() != shooterPIDConfigs.kP
				|| shooterKd.getNumber() != shooterPIDConfigs.kD
				|| shooterKv.getNumber() != shooterPIDConfigs.kV) {
			shooterPIDConfigs.kP = shooterKp.getNumber();
			shooterPIDConfigs.kD = shooterKd.getNumber();
			shooterPIDConfigs.kV = shooterKv.getNumber();
			m_ShooterMotor1.getConfigurator().apply(shooterPIDConfigs);
			m_ShooterMotor2.getConfigurator().apply(shooterPIDConfigs);
		}

		// lookup table stuff
		Pose2d targetHubPose = PeddieBounds.getHubTarget();

		// variable speeds
		if (ShooterTurretConstants.useVirtualTarget) {
			targetHubPose =
					ShooterPhysics.getVirtualTarget(
							RobotStates.robotPose.get(),
							RobotStates.fieldRelativeSpeeds.get(),
							ShooterTurretConstants.virtualTargetSolveIterations);
		}
		AngularVelocity velocity =
				ShooterPhysics.calculateShooterSpeed(RobotStates.robotPose.get(), targetHubPose);
		shooterTargetVelocity = velocity;

		if (m_ShooterState == ShooterStates.SHOOTING) {

			m_ShooterMotor1.setControl(m_VelocityRequest.withVelocity(velocity));
			m_ShooterMotor2.setControl(m_FollowerRequest);
		} else if (m_ShooterState == ShooterStates.DEFAULT) {
			m_ShooterMotor1.setControl(new CoastOut());
			m_ShooterMotor2.setControl(m_FollowerRequest);
		} else {
			m_ShooterMotor1.setControl(m_VelocityRequest.withVelocity(m_ShooterState.shooterSpeed));
			m_ShooterMotor2.setControl(m_FollowerRequest);
			shooterTargetVelocity = m_ShooterState.shooterSpeed;
		}
	}
}
