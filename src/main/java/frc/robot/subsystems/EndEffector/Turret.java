// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;
import static frc.robot.Constants.ShooterConstants.Turret.*;

import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants.Turret.MotorConfigs;
import frc.robot.state.RobotStates;
import java.util.Optional;

public class Turret extends SubsystemBase {

	private final TalonFX m_TurretMotor;

	private final DutyCycleEncoder m_TurretEncoder1;
	private final DutyCycleEncoder m_TurretEncoder2;

	private final MotionMagicExpoTorqueCurrentFOC turretRequest =
			new MotionMagicExpoTorqueCurrentFOC(Rotations.of(0));

	private boolean isReset = false;

	private Angle turretTargetAngleAbsolute = Degrees.of(0);
	private Angle turretTargetAngleMotor = Degrees.of(0);

	/** Creates a new Turret. */
	public Turret() {
		m_TurretMotor = new TalonFX(turretMotorCanID, superstructureCANBusName);

		m_TurretEncoder1 = new DutyCycleEncoder(encoder1Port, 1, encoder1Offset.in(Rotations));
		m_TurretEncoder2 = new DutyCycleEncoder(encoder2Port, 1, encoder2Offset.in(Rotations));

		m_TurretMotor.getConfigurator().apply(MotorConfigs.getTurretMotorConfig());

		turretRequest.UpdateFreqHz = 0;
		turretRequest.UseTimesync = true;
	}

	// given mechanism rotation, reset motor encoder to match
	public void resetMotorPosition() {
		Optional<Angle> turretPosition = getTurretHeadingCRT();
		if (turretPosition.isEmpty()) {
			return;
		}
		m_TurretMotor.setPosition(turretPosition.get().div(motorToMechanismRatio));
		isReset = true;
	}

	public double getMotorPositionRotations() {
		return m_TurretMotor.getPosition().getValue().in(Rotations);
	}

	public double getEncoder1PositionDegrees() {
		return m_TurretEncoder1.get() * 360.0;
	}

	public double getEncoder2PositionDegrees() {
		return m_TurretEncoder2.get() * 360.0;
	}

	// -135 deg = CW limit, 135 deg = CCW limit (CCW positive)
	// robot relative heading
	public Optional<Angle> getTurretHeadingCRT() {
		Angle encoder1Position = Degrees.of(getEncoder1PositionDegrees());
		Angle encoder2Position = Degrees.of(getEncoder2PositionDegrees());

		Angle possibleMechRot =
				Rotations.of(encoder1Position.in(Rotations) * encoderPinion1Teeth / ringGearTeeth);

		// calculate minimum possible solution for encoder 1 (closest to CW limit)
		possibleMechRot =
				Rotations.of(
						MathUtil.inputModulus(
								possibleMechRot.in(Rotations), 0, encoderPinion1Teeth / ringGearTeeth));

		// iterate through possible encoder 2 solutions
		Angle bestErr = Rotations.of(Double.MAX_VALUE);
		Angle secondErr = Rotations.of(Double.MAX_VALUE);
		Angle bestRot = Rotations.of(0);
		while (possibleMechRot.lte(turretRingGearRange)) {
			Angle encoder2Solution =
					Rotations.of((possibleMechRot.in(Rotations) * ringGearTeeth / encoderPinion2Teeth) % 1.0);

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

			possibleMechRot = possibleMechRot.plus(Rotations.of(encoderPinion1Teeth / ringGearTeeth));
		}

		// no solution found
		if (bestErr.in(Rotations) == Double.MAX_VALUE || bestErr.gt(matchTolerance)) {
			return Optional.empty();
		}

		// ambiguous solutions
		if (secondErr.lt(matchTolerance)
				&& Math.abs(secondErr.in(Rotations) - bestErr.in(Rotations))
						< ambiguityTolerance.in(Rotations)) {
			return Optional.empty();
		}

		// convert range from [0, 270] to [-135, 135]
		return Optional.of(bestRot.plus(turretRingGearRange.div(2.0)));
	}

	public Angle getTurretHeadingFromMotor() {
		return Rotations.of(getMotorPositionRotations() * motorToMechanismRatio);
	}

	public Pose2d getTurretPose() {
		Pose2d pose = RobotStates.robotPose.get();
		if (!isReset) {
			return pose;
		}
		Rotation2d robotHeading = pose.getRotation();

		Angle turretHeading = getTurretHeadingFromMotor();
		Translation2d translation =
				pose.getTranslation()
						.plus(new Translation2d(turretPositionOffset.getNorm(), robotHeading.plus(turretPositionOffset.getAngle())));
		Rotation2d rotation = pose.getRotation().plus(new Rotation2d(turretHeading));
		return new Pose2d(translation, rotation);
	}

	public void updateTurretTarget() {
		if (!isReset) {
			return;
		}
		Pose2d turretPose = getTurretPose();
		Pose2d targetHubPose =
				DriverStation.getAlliance().isPresent()
								&& DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
						? blueHub
						: redHub;
		Rotation2d fieldRelativeToHub =
				new Rotation2d(
						targetHubPose.getTranslation().getX() - turretPose.getTranslation().getX(),
						targetHubPose.getTranslation().getY() - turretPose.getTranslation().getY());
		Angle turretTarget =
				fieldRelativeToHub.getMeasure().minus(RobotStates.robotHeading.get().getMeasure());
		double angleDeg = turretTarget.in(Degrees); // (-180,180)
		angleDeg = (angleDeg < 0) ? (360 - Math.abs(angleDeg) % 360) % 360 : (angleDeg % 360);
		angleDeg -= 180;
		SmartDashboard.putNumber("angleDeg", angleDeg);
		if (Math.abs(angleDeg) > turretRingGearRange.in(Degrees) / 2.0) {
			double turnLimit = turretRingGearRange.in(Degrees) / 2.0;
			// Map [135, 180] -> [135, 0] linearly
			if (angleDeg > turnLimit) { // (135, 180]
				double t = (angleDeg - turnLimit) / (180 - turnLimit); // 0..1
				angleDeg = turnLimit * (1.0 - t) - 180; // 135..0
			} else {
				// Map [-180, -135] -> [0, -135] linearly
				// angleDeg in [-180, -135)
				double t = (angleDeg + 180.0) / (180 - turnLimit); // 0..1
				angleDeg = -turnLimit * t + 180;
			}
			turretTargetAngleAbsolute = Degrees.of(angleDeg);
			turretTargetAngleMotor = turretTargetAngleAbsolute.div(motorToMechanismRatio);
			return;
		}
		turretTargetAngleAbsolute = turretTarget;
		turretTargetAngleMotor = turretTargetAngleAbsolute.div(motorToMechanismRatio);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if (!isReset) {
			resetMotorPosition();
			return;
		}

		updateTurretTarget();

		m_TurretMotor.setControl(turretRequest.withPosition(turretTargetAngleMotor));
	}
}
