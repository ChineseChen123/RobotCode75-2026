// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;
import static frc.robot.Constants.ShooterTurretConstants.TurretConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;
import frc.lib.util.PeddieBounds;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;
import frc.lib.util.ShooterPhysics;
import frc.lib.util.ShooterPhysics.TurretSetpoint;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterTurretConstants;
import frc.robot.Constants.ShooterTurretConstants.TurretConstants;
import frc.robot.Constants.ShooterTurretConstants.TurretConstants.MotorConfigs;
import frc.robot.state.RobotStates;

public class Turret extends SubsystemBase {

	// ── State enums / result containers ──────────────────────────────────────────

	public enum TurretStates {
		STOWED,
		IDLE,
		SCORING;
	}

	public class CRTResult {
		public Angle turretAngle;
		public int quality; // 0 = no solution, 1 = acceptable, 2 = good

		public CRTResult(Angle turretAngle, int quality) {
			this.turretAngle = turretAngle;
			this.quality = quality;
		}
	}

	// ── Hardware ─────────────────────────────────────────────────────────────────

	private final TalonFX m_TurretMotor;
	private final DutyCycleEncoder m_TurretEncoder1;
	private final DutyCycleEncoder m_TurretEncoder2;

	// ── Control requests / configs ───────────────────────────────────────────────

	private final PositionVoltage turretRequest = new PositionVoltage(Rotations.of(0));
	private final Slot0Configs turretConfigs = new Slot0Configs();

	// ── Tunables ─────────────────────────────────────────────────────────────────

	private TunableNumber turretP;
	private TunableNumber turretD;
	private TunableNumber turretS;
	private TunableNumber turretV;
	private double kV = MotorConfigs.turretKV;

	// ── Internal state ───────────────────────────────────────────────────────────

	private TurretStates m_TurretState;
	private int resetState = 0; // 0 = not reset, 1 = acceptable, 2 = good

	private Angle turretTargetAngle = Degrees.of(0);
	private AngularVelocity turretTargetVelocity = RotationsPerSecond.of(0);

	private Angle currentTurretMotorPosition = Degrees.of(0);
	private AngularVelocity currentTurretVelocity = RotationsPerSecond.of(0);
	private double currentTurretErrorDegrees = 0;

	/** Creates a new Turret. */
	public Turret() {
		m_TurretMotor = new TalonFX(turretMotorCanID, superstructureCANBusName);

		// m_TurretState = TurretStates.STOWED;
		m_TurretState = TurretStates.IDLE;

		m_TurretEncoder1 = new DutyCycleEncoder(encoder1Port, 1, 0);
		m_TurretEncoder2 = new DutyCycleEncoder(encoder2Port, 1, 0);

		m_TurretMotor.getConfigurator().apply(MotorConfigs.getTurretMotorConfig());

		turretConfigs
				.withKP(MotorConfigs.turretKP)
				.withKD(MotorConfigs.turretKD)
				.withKS(MotorConfigs.turretKS);

		initTunables();

		turretRequest.UpdateFreqHz = 0;
		turretRequest.UseTimesync = true;
	}

	// ── Reset / initialization helpers ───────────────────────────────────────────

	@Logged(key = "Turret Reset State", importance = Importance.DEBUG)
	public int resetState() {
		return resetState;
	}

	/** Given mechanism rotation from CRT, reset motor encoder to match. */
	public void resetMotorPosition() {
		CRTResult result = getTurretHeadingCRT();
		if (result.quality == 0) {
			return;
		}

		m_TurretMotor.setPosition(result.turretAngle);
		System.out.println("Turret reset to " + result.turretAngle.in(Degrees) + " deg");
		resetState = result.quality;
	}

	// ── Motor / encoder accessors ────────────────────────────────────────────────

	@Logged(key = "Turret Position Deg", importance = Importance.DEBUG)
	public double getPositionFromMotorDegrees() {
		return currentTurretMotorPosition.in(Degrees);
	}

	@Logged(key = "Turret In Deadzone", importance = Importance.INFO)
	public boolean inDeadzone() {
		return ShooterPhysics.isTurretInDeadzone(
				RobotStates.robotPose.get(), RobotStates.fieldRelativeSpeeds.get());
	}

	public Angle getPositionFromMotor() {
		return currentTurretMotorPosition;
	}

	public AngularVelocity getTurretVelocity() {
		return currentTurretVelocity;
	}

	@Logged(key = "Encoder 1 Position Deg No Offset", importance = Importance.CRITICAL)
	public double getEncoder1PositionDegrees() {
		return m_TurretEncoder1.get() * 360.0;
	}

	@Logged(key = "Encoder 2 Position Deg No Offset", importance = Importance.CRITICAL)
	public double getEncoder2PositionDegrees() {
		return m_TurretEncoder2.get() * 360.0;
	}

	public double getEncoder1PositionDegreesWithOffset() {
		return MathUtil.inputModulus(
				getEncoder1PositionDegrees() - encoder1ZeroPoint.in(Degrees), 0, 360);
	}

	public double getEncoder2PositionDegreesWithOffset() {
		return MathUtil.inputModulus(
				getEncoder2PositionDegrees() - encoder2ZeroPoint.in(Degrees), 0, 360);
	}

	// ── CRT solve ────────────────────────────────────────────────────────────────

	/**
	 * Solves turret heading from the two absolute encoders.
	 *
	 * <p>Range convention: -135 deg = CW limit, +135 deg = CCW limit.
	 *
	 * <p>Returns robot-relative heading.
	 */
	public CRTResult getTurretHeadingCRT() {
		Angle encoder1Position = Degrees.of(getEncoder1PositionDegreesWithOffset());
		Angle encoder2Position = Degrees.of(getEncoder2PositionDegreesWithOffset());

		encoder1Position = Rotations.of(MathUtil.inputModulus(encoder1Position.in(Rotations), 0, 1));
		encoder2Position = Rotations.of(MathUtil.inputModulus(encoder2Position.in(Rotations), 0, 1));

		Angle possibleMechRot =
				Rotations.of(encoder1Position.in(Rotations) * encoderPinion1Teeth / ringGearTeeth);

		// Minimum possible solution for encoder 1 (closest to 0 / CW limit)
		possibleMechRot =
				Rotations.of(
						MathUtil.inputModulus(
								possibleMechRot.in(Rotations), 0, encoderPinion1Teeth / ringGearTeeth));

		Angle bestErr = Rotations.of(Double.MAX_VALUE);
		Angle secondErr = Rotations.of(Double.MAX_VALUE);
		Angle bestRot = Rotations.of(0);

		while (possibleMechRot.lte(turretRange)) {
			Angle encoder2Solution =
					Rotations.of(
							MathUtil.inputModulus(
									possibleMechRot.in(Rotations) * ringGearTeeth / encoderPinion2Teeth, 0, 1));

			Angle err = Rotations.of(encoder2Position.minus(encoder2Solution).abs(Rotations));
			if (err.gt(Rotations.of(0.5))) {
				err = Rotations.of(1.0).minus(err);
			}

			if (err.lt(bestErr)) {
				secondErr = bestErr;
				bestErr = err;
				bestRot = possibleMechRot;
			} else if (err.lt(secondErr)) {
				secondErr = err;
			}

			possibleMechRot =
					possibleMechRot.plus(Rotations.of(encoderPinion1Teeth * 1.0 / ringGearTeeth));
		}

		// No solution found
		if (!Double.isFinite(bestErr.in(Rotations)) || bestErr.gt(acceptableMatchTolerance)) {
			System.out.println("Best error:" + bestErr.in(Degrees));
			return new CRTResult(null, 0);
		}

		// Acceptable but not good
		if (bestErr.gt(goodMatchTolerance)) {
			return new CRTResult(bestRot.minus(turretRange.div(2)), 1);
		}

		// Ambiguous solutions
		if (secondErr.lte(goodMatchTolerance)
				&& Math.abs(secondErr.in(Rotations) - bestErr.in(Rotations))
						< ambiguityTolerance.in(Rotations)) {
			return new CRTResult(bestRot.minus(turretRange.div(2)), 1);
		}

		// Convert range from [0, 270] to [-135, +135]
		return new CRTResult(bestRot.minus(turretRange.div(2)), 2);
	}

	// ── Target / state API ───────────────────────────────────────────────────────

	@Logged(key = "Turret Position Error Deg", importance = Importance.DEBUG)
	public double getTurretPositionErrorDegrees() {
		return currentTurretErrorDegrees;
	}

	public boolean atTargetHeading() {
		return currentTurretErrorDegrees < turretPositionToleranceDegrees;
	}

	@Logged(key = "Turret Target", importance = Importance.DEBUG)
	public double getTurretTargetDegrees() {
		return turretTargetAngle.in(Degrees);
	}

	@Logged(key = "Turret Distance", importance = Importance.DEBUG)
	public double getTurretDistance() {
		return getTurretPose().getTranslation().getDistance(getShootingTargetPose().getTranslation());
	}

	@Logged(key = "Turret Target Velocity DPS", importance = Importance.DEBUG)
	public double getTurretTargetVelocityDPS() {
		return turretTargetVelocity.in(DegreesPerSecond);
	}

	public TurretStates getTurretState() {
		return m_TurretState;
	}

	public void setState(TurretStates state) {
		m_TurretState = state;
	}

	public boolean isStowed() {
		return getTurretState() == TurretStates.STOWED && atTargetHeading();
	}

	// ── Pose / targeting ─────────────────────────────────────────────────────────

	@Logged(key = "Turret Pose", importance = Importance.CRITICAL)
	public Pose2d getTurretPose() {
		Pose2d pose = RobotStates.robotPose.get();
		if (resetState == 0) {
			return pose;
		}

		Rotation2d robotHeading = pose.getRotation();

		Translation2d translation = pose.transformBy(turretPositionOffset).getTranslation();
		Rotation2d rotation = robotHeading.plus(new Rotation2d(currentTurretMotorPosition));

		return new Pose2d(translation, rotation);
	}

	@Logged(key = "Shooting Target Pose", importance = Importance.DEBUG)
	public Pose2d getShootingTargetPose() {
		return ShooterTurretConstants.useVirtualTarget
				? ShooterPhysics.getVirtualTarget(
						RobotStates.robotPose.get(),
						RobotStates.fieldRelativeSpeeds.get(),
						ShooterTurretConstants.virtualTargetSolveIterations)
				: PeddieBounds.getShootingTargetPose(RobotStates.robotPose.get()).toPose2d();
	}

	/** Updates turret target angle/velocity from shooter physics. */
	public void updateTurretTarget() {
		if (resetState == 0) {
			return;
		}

		TurretSetpoint setpoint =
				ShooterPhysics.calculateTurretSetpoint(
						RobotStates.robotPose.get(), RobotStates.fieldRelativeSpeeds.get());

		turretTargetAngle = setpoint.turretAngle;
		turretTargetVelocity = setpoint.turretVelocity;
	}

	public void updateCache() {
		currentTurretMotorPosition = m_TurretMotor.getPosition(true).getValue();
		currentTurretVelocity = m_TurretMotor.getVelocity(true).getValue();
		currentTurretErrorDegrees = turretTargetAngle.minus(currentTurretMotorPosition).in(Degrees);
	}

	// ── Commands ─────────────────────────────────────────────────────────────────

	public Command setStateCommand(TurretStates state) {
		return new InstantCommand(() -> setState(state), this)
				.repeatedly()
				.finallyDo(() -> setState(TurretStates.IDLE));
	}

	public Command setStateCommandPersistent(TurretStates state) {
		return new InstantCommand(() -> setState(state), this);
	}

	// ── WPILib lifecycle ─────────────────────────────────────────────────────────

	@Override
	public void periodic() {
		if (resetState < 1 && getTurretVelocity().abs(RotationsPerSecond) < 0.005) {
			resetMotorPosition();
			if (resetState == 0) {
				return;
			}
		}

		updateTunables();
		updateCache();

		switch (m_TurretState) {
			case STOWED:
				m_TurretMotor.setControl(turretRequest.withPosition(TurretConstants.turretStowAngle));
				break;
			case IDLE:
				updateTurretTarget();
				m_TurretMotor.setControl(new StaticBrake());
				break;
			case SCORING:
				updateTurretTarget();
				m_TurretMotor.setControl(turretRequest.withPosition(turretTargetAngle).withFeedForward(kV * RobotStates.fieldRelativeSpeeds.get().omegaRadiansPerSecond * 0.5 / Math.PI));
				break;
		}
	}

	// ── Tuning ───────────────────────────────────────────────────

	public void initTunables() {
		if (RobotConstants.TuningModes.tuneTurret) {
			turretP = new TunableNumber("Turret/kP", MotorConfigs.turretKP);
			turretD = new TunableNumber("Turret/kD", MotorConfigs.turretKD);
			turretS = new TunableNumber("Turret/kS", MotorConfigs.turretKS);
			turretV = new TunableNumber("Turret/kV", MotorConfigs.turretKV);
		}
	}

	private void updateTunables() {
		if (RobotConstants.TuningModes.tuneTurret
				&& (turretP.getNumber() != turretConfigs.kP
						|| turretD.getNumber() != turretConfigs.kD
						|| turretS.getNumber() != turretConfigs.kS
						|| turretV.getNumber() != kV)) {
			turretConfigs.kP = turretP.getNumber();
			turretConfigs.kD = turretD.getNumber();
			turretConfigs.kS = turretS.getNumber();
			kV = turretV.getNumber();
			m_TurretMotor.getConfigurator().apply(turretConfigs);
		}
	}
}
