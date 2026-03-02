package frc.lib.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ShooterTurretConstants.ShooterConstants.*;
import static frc.robot.Constants.ShooterTurretConstants.TurretConstants.turretPositionOffset;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.FieldConstants;
import frc.robot.state.RobotStates;

public class ShooterPhysics {

	public static AngularVelocity distanceToMotorVelocity(double distanceToHub) {
		if (distanceToHub < 2.1082) {
			return RotationsPerSecond.of(2650 / 60);
		}
		return RotationsPerSecond.of(
				Math.min(
								(shooterRegressionA * Math.sqrt(distanceToHub - shooterRegressionC)
										+ shooterRegressionB),
								4000)
						/ 60.0);
	}

	public static AngularVelocity calculateShooterSpeed(Pose2d robotPose, Pose2d targetHubPose) {
		Translation2d turretPose = robotPose.transformBy(turretPositionOffset).getTranslation();
		return distanceToMotorVelocity(turretPose.getDistance(targetHubPose.getTranslation()));
	}

	public static LinearVelocity motorVelocityToLinearVelocity(AngularVelocity shooterVelocity) {
		double totalMOI = flywheelMOI + shooterWheelMOI; // in^2 / lbs
		double shooterWheelRadiusInches = (shooterWheelDiameter.in(Inches) / 2);
		LinearVelocity surfaceWheelSpeed =
				InchesPerSecond.of(
						shooterVelocity.in(RadiansPerSecond) * shooterGearRatio * shooterWheelRadiusInches);

		// https://www.reca.lc/flywheel
		double speedTransferPercentage =
				(20 * totalMOI)
						/ (7 * ballWeight * shooterWheelRadiusInches * shooterWheelRadiusInches / 2
								+ 40 * totalMOI);
		LinearVelocity projectileSpeed = surfaceWheelSpeed.times(speedTransferPercentage);
		return projectileSpeed;
	}

	public static AngularVelocity linearVelocityToMotorVelocity(LinearVelocity projectileSpeed) {

		double totalMOI = flywheelMOI + shooterWheelMOI; // in^2 / lbs
		double shooterWheelRadiusInches = (shooterWheelDiameter.in(Inches) / 2);
		double speedTransferPercentage =
				(20 * totalMOI)
						/ (7 * ballWeight * shooterWheelRadiusInches * shooterWheelRadiusInches / 2
								+ 40 * totalMOI);

		double surfaceSpeedInchesPerSecond =
				projectileSpeed.in(InchesPerSecond) / speedTransferPercentage;

		double angularVelocityRadPerSec = surfaceSpeedInchesPerSecond / shooterWheelRadiusInches;

		return RadiansPerSecond.of(angularVelocityRadPerSec / shooterGearRatio);
	}

	private static final double phaseDelay = 0.03;

	public static Time calculateTimeToScore(
			Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, Pose2d targetHubPose) {

		ChassisSpeeds robotRelativeSpeeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, RobotStates.robotHeading.get());
		robotPose =
				robotPose.exp(
						new Twist2d(
								robotRelativeSpeeds.vxMetersPerSecond * phaseDelay,
								robotRelativeSpeeds.vyMetersPerSecond * phaseDelay,
								robotRelativeSpeeds.omegaRadiansPerSecond * phaseDelay));

		Translation2d turretPose = robotPose.transformBy(turretPositionOffset).getTranslation();

		AngularVelocity shooterVelocity =
				distanceToMotorVelocity(turretPose.getDistance(targetHubPose.getTranslation()));
		LinearVelocity projectileSpeed = motorVelocityToLinearVelocity(shooterVelocity);

		// gravity in inches/sec^2
		double g = 386.09;
		LinearVelocity yComponent =
				projectileSpeed.times(Math.sin(shooterAngleWithVertical.in(Radians)));
		double yComponentInches = yComponent.in(InchesPerSecond);
		double heightDiffInches =
				FieldConstants.hubEntranceHeight.in(Inches) - shooterHeight.in(Inches);

		double secondsToScore =
				(yComponentInches
								+ Math.sqrt(yComponentInches * yComponentInches + 2 * g * (heightDiffInches)))
						/ g;

		return Seconds.of(secondsToScore);
	}

	public static Pose2d getVirtualTarget(
			Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, int iterations) {

		Pose2d virtualTargetPose = PeddieBounds.getHubTarget();

		Rotation2d robotAngle = robotPose.getRotation();
		double turretVelocityX =
				fieldRelativeSpeeds.vxMetersPerSecond
						+ fieldRelativeSpeeds.omegaRadiansPerSecond
								* (turretPositionOffset.getY() * robotAngle.getCos()
										- turretPositionOffset.getX() * robotAngle.getSin());
		double turretVelocityY =
				fieldRelativeSpeeds.vyMetersPerSecond
						+ fieldRelativeSpeeds.omegaRadiansPerSecond
								* (turretPositionOffset.getX() * robotAngle.getCos()
										- turretPositionOffset.getY() * robotAngle.getSin());

		for (int i = 0; i < iterations; i++) {
			Time tofEstimate = calculateTimeToScore(robotPose, fieldRelativeSpeeds, virtualTargetPose);

			Translation2d targetTranslation =
					new Translation2d(
							MetersPerSecond.of(-turretVelocityX).times(tofEstimate),
							MetersPerSecond.of(-turretVelocityY).times(tofEstimate));
			virtualTargetPose =
					virtualTargetPose.plus(new Transform2d(targetTranslation, Rotation2d.kZero));
		}
		return virtualTargetPose;
	}
}
