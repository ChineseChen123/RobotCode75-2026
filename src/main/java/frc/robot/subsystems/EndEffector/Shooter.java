// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;
import static frc.robot.Constants.ShooterTurretConstants.ShooterConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;
import frc.lib.util.PeddieBounds;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;
import frc.lib.util.ShooterPhysics;
import frc.robot.Constants.ShooterTurretConstants;
import frc.robot.Constants.ShooterTurretConstants.ShooterConstants.MotorConfigs;
import frc.robot.state.RobotStates;

public class Shooter extends SubsystemBase {

	// ── State enum ───────────────────────────────────────────────────────────────

	public enum ShooterStates {
		DEFAULT(defaultShooterSpeed),
		SHOOTING(null),
		REVERSING(reverseShooterSpeed);

		AngularVelocity shooterSpeed;

		private ShooterStates(AngularVelocity shooterSpeed) {
			this.shooterSpeed = shooterSpeed;
		}
	}

	// ── Hardware ─────────────────────────────────────────────────────────────────

	private final TalonFX m_ShooterMotor1;
	private final TalonFX m_ShooterMotor2;

	// ── Control requests ─────────────────────────────────────────────────────────

	private final VelocityTorqueCurrentFOC m_VelocityRequest;
	private final Follower m_FollowerRequest;

	private final VelocityDutyCycle m_DutyCycleBangBang;
	private final VelocityTorqueCurrentFOC m_TorqueCurrentBangBang;

	// ── Tunables / configs ───────────────────────────────────────────────────────

	private final Slot0Configs shooterPIDConfigs = new Slot0Configs();

	// private final TunableNumber shooterKp;
	// private final TunableNumber shooterKd;
	// private final TunableNumber shooterKv;
	// private final TunableNumber shooterKa;
	// private final TunableNumber targetSpeed;

	// ── Internal state ───────────────────────────────────────────────────────────

	private ShooterStates m_ShooterState;
	private AngularVelocity shooterTargetVelocity = RPM.of(0);
	private AngularVelocity currentShooterVelocity = RPM.of(0);

	private final Debouncer atSetpointDebouncer = new Debouncer(0.025, DebounceType.kFalling);
	private boolean lastAtSetpoint = false;

	private int shotsFired = 0;

	/** Creates a new Shooter. */
	public Shooter() {
		m_ShooterMotor1 = new TalonFX(shooterMotor1CanID, superstructureCANBusName);
		m_ShooterMotor2 = new TalonFX(shooterMotor2CanID, superstructureCANBusName);

		m_ShooterMotor1.getConfigurator().apply(MotorConfigs.getShooterBangBangConfiguration());
		m_ShooterMotor2.getConfigurator().apply(MotorConfigs.getShooterBangBangConfiguration());

		m_VelocityRequest = new VelocityTorqueCurrentFOC(RPM.of(0));
		m_FollowerRequest = new Follower(m_ShooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed);

		m_DutyCycleBangBang = new VelocityDutyCycle(RPM.of(0));
		m_TorqueCurrentBangBang = new VelocityTorqueCurrentFOC(RPM.of(0));

		m_ShooterState = ShooterStates.DEFAULT;

		shooterPIDConfigs
				.withKP(MotorConfigs.shooterMotorVelocityKP)
				.withKI(MotorConfigs.shooterMotorVelocityKI)
				.withKD(MotorConfigs.shooterMotorVelocityKD)
				.withKS(MotorConfigs.shooterMotorVelocityKS)
				.withKV(MotorConfigs.shooterMotorVelocityKV)
				.withKA(MotorConfigs.shooterMotorVelocityKA);

		// shooterKp = new TunableNumber("Shooter/Kp", 10000);
		// shooterKd = new TunableNumber("Shooter/Kd", MotorConfigs.shooterMotorVelocityKD);
		// shooterKv = new TunableNumber("Shooter/Kv", MotorConfigs.shooterMotorVelocityKV);
		// shooterKa = new TunableNumber("Shooter/Ka", MotorConfigs.shooterMotorVelocityKA);
		// targetSpeed = new TunableNumber("Shooter/Target Speed RPM", 0);

		m_VelocityRequest.UpdateFreqHz = 0;
		m_VelocityRequest.UseTimesync = true;
	}

	// ── Velocity / state accessors ───────────────────────────────────────────────

	@Logged(key = "Shooter Motor Velocity RPM", importance = Importance.CRITICAL)
	public double getMotorVelocityRPM() {
		return currentShooterVelocity.in(RPM);
	}

	@Logged(key = "Shooter Wheel Velocity RPM", importance = Importance.CRITICAL)
	public double getWheelVelocityRPM() {
		return getMotorVelocityRPM() * shooterGearRatio;
	}

	public AngularVelocity getVelocity() {
		return m_ShooterMotor1
				.getVelocity(true)
				.getValue()
				.plus(m_ShooterMotor2.getVelocity(true).getValue().times(-1))
				.div(2);
	}

	@Logged(key = "Shooter Target Velocity RPM", importance = Importance.CRITICAL)
	public double targetVelocityRPM() {
		return shooterTargetVelocity.in(RPM);
	}

	public boolean atTargetVelocity() {
		return currentShooterVelocity.minus(shooterTargetVelocity).abs(RPM)
				< shooterVelocityTolerance;
	}

	public boolean aboveTargetVelocity() {
		return currentShooterVelocity.in(RPM) > shooterTargetVelocity.in(RPM);
	}

	public void updateCache() {
		currentShooterVelocity = getVelocity();
	}

	public ShooterStates getShooterState() {
		return m_ShooterState;
	}

	public void setState(ShooterStates state) {
		m_ShooterState = state;
	}

	@Override
	public void periodic() {

		updateCache();

		// if (m_ShooterState == ShooterStates.SHOOTING) {
		// 	m_ShooterMotor1.setControl(
		// 			m_VelocityRequest.withVelocity(RotationsPerSecond.of(targetSpeed.getNumber() / 60)));
		// 	m_ShooterMotor2.setControl(m_FollowerRequest);
		// 	shooterTargetVelocity = RotationsPerSecond.of(targetSpeed.getNumber() / 60);
		// } else {
		// 	m_ShooterMotor1.setControl(new CoastOut());
		// 	m_ShooterMotor2.setControl(m_FollowerRequest);
		// }

		// if (shooterKp.getNumber() != shooterPIDConfigs.kP
		// 		|| shooterKd.getNumber() != shooterPIDConfigs.kD
		// 		|| shooterKv.getNumber() != shooterPIDConfigs.kV
		// 		|| shooterKa.getNumber() != shooterPIDConfigs.kA) {
		// 	shooterPIDConfigs.kP = shooterKp.getNumber();
		// 	// shooterPIDConfigs.kD = shooterKd.getNumber();
		// 	// shooterPIDConfigs.kV = shooterKv.getNumber();
		// 	// shooterPIDConfigs.kA = shooterKa.getNumber();
		// 	m_ShooterMotor1.getConfigurator().apply(shooterPIDConfigs);
		// 	m_ShooterMotor2.getConfigurator().apply(shooterPIDConfigs);
		// }

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

		// shooterTargetVelocity = RotationsPerSecond.of(targetSpeed.getNumber() / 60);

		boolean debouncedAtSetpoint = atSetpointDebouncer.calculate(atTargetVelocity());

		if (m_ShooterState == ShooterStates.SHOOTING) {
			// m_ShooterMotor1.setControl(m_VelocityRequest.withVelocity(velocity));
			if (debouncedAtSetpoint) {
				m_ShooterMotor1.setControl(m_TorqueCurrentBangBang.withVelocity(shooterTargetVelocity));
			} else {
				m_ShooterMotor1.setControl(m_DutyCycleBangBang.withVelocity(shooterTargetVelocity));
			}

			// shooterTargetVelocity = m_ShooterState.shooterSpeed;

		} else if (m_ShooterState == ShooterStates.DEFAULT) {
			m_ShooterMotor1.setControl(new CoastOut());
			shooterTargetVelocity = RPM.of(0);
		} else {
			m_ShooterMotor1.setControl(m_VelocityRequest.withVelocity(m_ShooterState.shooterSpeed));
			shooterTargetVelocity = m_ShooterState.shooterSpeed;
		}

		// fuel counting
		if (m_ShooterState == ShooterStates.SHOOTING) {
			if (!debouncedAtSetpoint && lastAtSetpoint) {
				shotsFired++;
			}
			lastAtSetpoint = debouncedAtSetpoint;
		} else {
			lastAtSetpoint = false;
		}

		m_ShooterMotor2.setControl(m_FollowerRequest);
	}
}