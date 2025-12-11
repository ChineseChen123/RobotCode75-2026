// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Swerve;

/** Add your docs here. */
public class ChezyController {

	// private final TunableNumber driveP = new TunableNumber("ChezyController/Drive P", 2);
	// private final TunableNumber rotationP = new TunableNumber("ChezyController/Rotation P", 3);

	private final Swerve m_swerve;
	private final ProfiledPIDController driveController =
			new ProfiledPIDController(
					DrivetrainConstants.ControllerConstants.OdometryAlign.xP,
					0.0,
					0.0,
					new TrapezoidProfile.Constraints(0.0, 0.0),
					0.02);
	private final ProfiledPIDController thetaController =
			new ProfiledPIDController(
					DrivetrainConstants.ControllerConstants.OdometryAlign.tP,
					0.0,
					0.0,
					new TrapezoidProfile.Constraints(0.0, 0.0),
					0.02);

	private Translation2d lastSetpointTranslation;
	private double driveErrorAbs;
	private double thetaErrorAbs;
	private double ffMinRadius = 0.2, ffMaxRadius = 0.8;

	private boolean rotationFinished = false;

	private double currentRotation;
	private double targetRotation;

	private Pose2d target;

	public ChezyController() {
		m_swerve = RobotContainer.getSwerve();
	}

	public void reset(Pose2d targetPose) {
		Pose2d currentPose = m_swerve.getPose();
		driveController.reset(
				currentPose.getTranslation().getDistance(targetPose.getTranslation()),
				Math.min(
						0.0,
						-new Translation2d(
										m_swerve.getChassisSpeeds().vxMetersPerSecond,
										m_swerve.getChassisSpeeds().vyMetersPerSecond)
								.rotateBy(
										targetPose
												.getTranslation()
												.minus(currentPose.getTranslation())
												.getAngle()
												.unaryMinus())
								.getX()));
		thetaController.reset(
				currentPose.getRotation().getRadians(), m_swerve.getChassisSpeeds().omegaRadiansPerSecond);
		driveController.setTolerance(DrivetrainConstants.ControllerConstants.toleranceTranslation);
		thetaController.setTolerance(DrivetrainConstants.ControllerConstants.toleranceRadians);
		lastSetpointTranslation = currentPose.getTranslation();

		rotationFinished = false;

		driveController.setGoal(0.0);
		thetaController.setGoal(0.0);
	}

	private double wrap(double angle) {
		if (angle < -Math.PI) {
			return angle + 2 * Math.PI;
		}
		if (angle > Math.PI) {
			return angle - 2 * Math.PI;
		}
		return angle;
	}

	public ChassisSpeeds update(Pose2d targetPose) {

		Pose2d currentPose = m_swerve.getPose();

		target = targetPose;

		double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
		double ffScaler =
				MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 0.5);
		driveErrorAbs = currentDistance;
		driveController.reset(
				lastSetpointTranslation.getDistance(targetPose.getTranslation()),
				driveController.getSetpoint().velocity);
		double driveVelocityScalar =
				driveController.getSetpoint().velocity * ffScaler
						+ driveController.calculate(driveErrorAbs, 0.0);
		if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
		lastSetpointTranslation =
				new Pose2d(
								targetPose.getTranslation(),
								currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
						.transformBy(
								new Transform2d(
										new Translation2d(driveController.getSetpoint().position, 0.0),
										new Rotation2d()))
						.getTranslation();

		// Calculate theta speed
		double thetaVelocity =
				thetaController.getSetpoint().velocity * ffScaler * 0.3
						+ thetaController.calculate(
								wrap(
										currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians()),
								0.0);
		currentRotation = wrap(currentPose.getRotation().getRadians());
		targetRotation = wrap(targetPose.getRotation().getRadians());
		thetaErrorAbs =
				Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
		if (thetaErrorAbs < thetaController.getPositionTolerance()) {
			rotationFinished = true;
			thetaVelocity = 0.0;
		}
		// Command speeds
		Translation2d driveVelocity =
				new Pose2d(0, 0, currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
						.transformBy(
								new Transform2d(new Translation2d(driveVelocityScalar, 0.0), new Rotation2d()))
						.getTranslation();
		return new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity);
	}

	public boolean isRotationFinished() {
		return rotationFinished;
	}

	public boolean isFinished() {
		return driveController.atGoal() && thetaController.atGoal();
	}
}
