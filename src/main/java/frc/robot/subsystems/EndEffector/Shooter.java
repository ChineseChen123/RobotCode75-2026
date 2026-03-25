// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;
import static frc.robot.Constants.ShooterTurretConstants.ShooterConstants.*;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
		SHOOTING(null);

		public final AngularVelocity shooterSpeed;

		private ShooterStates(AngularVelocity shooterSpeed) {
			this.shooterSpeed = shooterSpeed;
		}
	}

	// ── Hardware ─────────────────────────────────────────────────────────────────

	private final TalonFX m_ShooterMotor1;
	private final TalonFX m_ShooterMotor2;

	// ── Control requests / configs ───────────────────────────────────────────────

	private final VelocityTorqueCurrentFOC m_VelocityRequest;
	private final Follower m_FollowerRequest;

	private final VelocityDutyCycle m_DutyCycleBangBang;
	private final VelocityTorqueCurrentFOC m_TorqueCurrentBangBang;

	// ── Internal state ───────────────────────────────────────────────────────────

	private ShooterStates m_ShooterState = ShooterStates.DEFAULT;

	private AngularVelocity shooterTargetVelocity = RPM.of(0);
	private AngularVelocity currentShooterVelocity = RPM.of(0);

	private final Debouncer atSetpointDebouncer = new Debouncer(0.025, DebounceType.kFalling);
	private boolean lastAtSetpoint = false;

	private int totalShotsFired = 0;
	private int shotsFired = 0;

	/** Creates a new Shooter. */
	public Shooter() {
		m_ShooterMotor1 = new TalonFX(shooterMotor1CanID, superstructureCANBusName);
		m_ShooterMotor2 = new TalonFX(shooterMotor2CanID, superstructureCANBusName);

		m_ShooterMotor1.getConfigurator().apply(MotorConfigs.getShooterBangBangConfiguration());
		m_ShooterMotor2.getConfigurator().apply(MotorConfigs.getShooterBangBangConfiguration());

		m_VelocityRequest = new VelocityTorqueCurrentFOC(RPM.of(0));
		m_FollowerRequest = new Follower(m_ShooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed);

		m_DutyCycleBangBang = new VelocityDutyCycle(RPM.of(0)).withEnableFOC(true);
		m_TorqueCurrentBangBang = new VelocityTorqueCurrentFOC(RPM.of(0));

		m_VelocityRequest.UpdateFreqHz = 50;
	}

	// ── Velocity / state accessors ───────────────────────────────────────────────

	@Logged(key = "Shooter Motor Velocity RPM", importance = Importance.CRITICAL)
	public double getMotorVelocityRPM() {
		return currentShooterVelocity.in(RPM);
	}

	@Logged(key = "Shooter Wheel Velocity RPM", importance = Importance.DEBUG)
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
		return shooterTargetVelocity.in(RPM);
	}

	public boolean atTargetVelocity() {
		return currentShooterVelocity.minus(shooterTargetVelocity).abs(RPM) < shooterVelocityTolerance;
	}

	public boolean aboveTargetVelocity() {
		return currentShooterVelocity.in(RPM)
				> (shooterTargetVelocity.in(RPM) - shooterVelocityTolerance);
	}

	public ShooterStates getShooterState() {
		return m_ShooterState;
	}

	public void setState(ShooterStates state) {
		m_ShooterState = state;
	}

	@Logged(key = "Shooter Shots Fired", importance = Importance.CRITICAL)
	public int getShotsFired() {
		return shotsFired;
	}

	// ── Target calculation / caching ─────────────────────────────────────────────

	public void updateCache() {
		currentShooterVelocity = getVelocity();
	}

	/** Updates shooter target velocity from shooter physics. */
	public void updateShooterTarget() {

		if (!RobotStates.turretIsAligning.getAsBoolean()
				|| (RobotStates.turretIsAligning.getAsBoolean()
						&& RobotStates.actionAimTurretHold.getAsBoolean())) {
			shooterTargetVelocity = minShootingAngularVelocity.plus(maxShootingAngularVelocity).div(2.0);
			return;
		}

		Pose2d targetPose = PeddieBounds.getShootingTargetPose(RobotStates.robotPose.get()).toPose2d();

		if (ShooterTurretConstants.useVirtualTarget) {
			targetPose =
					ShooterPhysics.getVirtualTarget(
							RobotStates.robotPose.get(),
							RobotStates.fieldRelativeSpeeds.get(),
							ShooterTurretConstants.virtualTargetSolveIterations);
		}

		shooterTargetVelocity =
				ShooterPhysics.calculateShooterSpeed(RobotStates.robotPose.get(), targetPose);
	}

	// ── Control helpers ──────────────────────────────────────────────────────────

	private boolean bangbangShooting() {
		updateShooterTarget();

		boolean debouncedAtSetpoint = atSetpointDebouncer.calculate(aboveTargetVelocity());

		m_ShooterMotor1.setControl(m_TorqueCurrentBangBang.withVelocity(shooterTargetVelocity));

		return debouncedAtSetpoint;
	}

	// ── Shot counting ────────────────────────────────────────────────────────────

	private void updateShotCounting(boolean debouncedAtSetpoint) {
		if (m_ShooterState == ShooterStates.SHOOTING) {
			if (!debouncedAtSetpoint && lastAtSetpoint) {
				shotsFired++;
				totalShotsFired++;
			}
			lastAtSetpoint = debouncedAtSetpoint;
		} else {
			lastAtSetpoint = false;
		}
	}

	// ── Commands ─────────────────────────────────────────────────────────────────

	public Command setStateCommand(ShooterStates state) {
		return new InstantCommand(() -> setState(state), this)
				.repeatedly()
				.finallyDo(() -> setState(ShooterStates.DEFAULT));
	}

	public Command shootXBallsCommand(int numBalls) {
		return new InstantCommand(() -> shotsFired = 0)
				.andThen(setStateCommand(ShooterStates.SHOOTING).until(() -> shotsFired >= numBalls));
	}

	public Command setStateCommandPersistent(ShooterStates state) {
		return new InstantCommand(() -> setState(state), this);
	}

	// ── WPILib lifecycle ─────────────────────────────────────────────────────────

	@Override
	public void periodic() {
		updateCache();

		boolean debouncedAtSetpoint = false;

		switch (m_ShooterState) {
			case SHOOTING:
				debouncedAtSetpoint = bangbangShooting();
				break;

			case DEFAULT:
				m_ShooterMotor1.setControl(new CoastOut());
				shooterTargetVelocity = RPM.of(0);
				break;
		}

		updateShotCounting(debouncedAtSetpoint);
		m_ShooterMotor2.setControl(m_FollowerRequest);
	}
}
