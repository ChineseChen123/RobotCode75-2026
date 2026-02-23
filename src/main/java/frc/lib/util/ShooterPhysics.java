package frc.lib.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
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
import frc.robot.Constants.ShooterTurretConstants;
import frc.robot.state.RobotStates;

public class ShooterPhysics {

	public static AngularVelocity distanceToAngularVelocity(double distanceToHub) {
		// calculate from best fit line
		// will probably be A * sqrt(d) + B
		// we should measure RPM -> Distance FIRST, get the quadratic relation (d = A(RPM - B)^2) and
		// then invert
		// we could also linearly interpolate over a lookup table
		return null;
	}

	public static AngularVelocity calculateShooterSpeed(Pose2d robotPose, Pose2d targetHubPose) {
		return distanceToAngularVelocity(
				robotPose.getTranslation().getDistance(targetHubPose.getTranslation()));
	}

	public static LinearVelocity shooterAngularVelocityToLinearVelocity(
			AngularVelocity shooterVelocity) {
		double totalMOI =
				ShooterTurretConstants.ShooterConstants.flywheelMOI
						+ ShooterTurretConstants.ShooterConstants.shooterWheelMOI; // in^2 / lbs
		double shooterWheelRadiusInches =
				(ShooterTurretConstants.ShooterConstants.shooterWheelDiameter.in(Inches) / 2);
		LinearVelocity surfaceWheelSpeed =
				InchesPerSecond.of(shooterVelocity.in(RadiansPerSecond) * shooterWheelRadiusInches);

		// https://www.reca.lc/flywheel
		double speedTransferPercentage =
				(20 * totalMOI)
						/ (7
										* ShooterTurretConstants.ShooterConstants.ballWeight
										* shooterWheelRadiusInches
										* shooterWheelRadiusInches
										/ 2
								+ 40 * totalMOI);
		LinearVelocity projectileSpeed = surfaceWheelSpeed.times(speedTransferPercentage);
		return projectileSpeed;
	}

	public static AngularVelocity linearVelocityToShooterAngularVelocity(
			LinearVelocity projectileSpeed) {

		double totalMOI =
				ShooterTurretConstants.ShooterConstants.flywheelMOI
						+ ShooterTurretConstants.ShooterConstants.shooterWheelMOI; // in^2 / lbs
		double shooterWheelRadiusInches =
				(ShooterTurretConstants.ShooterConstants.shooterWheelDiameter.in(Inches) / 2);
		double speedTransferPercentage =
				(20 * totalMOI)
						/ (7
										* ShooterTurretConstants.ShooterConstants.ballWeight
										* shooterWheelRadiusInches
										* shooterWheelRadiusInches
										/ 2
								+ 40 * totalMOI);

		double surfaceSpeedInchesPerSecond =
				projectileSpeed.in(InchesPerSecond) / speedTransferPercentage;

		double angularVelocityRadPerSec = surfaceSpeedInchesPerSecond / shooterWheelRadiusInches;

		return RadiansPerSecond.of(angularVelocityRadPerSec);
	}

	private static final double phaseDelay = 0.03;

	public static Time calculateTimeToScore(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, Pose2d targetHubPose) {

		ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
			fieldRelativeSpeeds,
			RobotStates.robotHeading.get()
		);
		robotPose = robotPose.exp(
            new Twist2d(
                robotRelativeSpeeds.vxMetersPerSecond * phaseDelay,
                robotRelativeSpeeds.vyMetersPerSecond * phaseDelay,
                robotRelativeSpeeds.omegaRadiansPerSecond * phaseDelay));

		Translation2d turretPose =
				robotPose.getTranslation()
						.plus(
								new Translation2d(
										turretPositionOffset.getNorm(),
										RobotStates.robotHeading.get().plus(turretPositionOffset.getAngle())));

		AngularVelocity shooterVelocity =
				distanceToAngularVelocity(
						turretPose.getDistance(targetHubPose.getTranslation()));
		LinearVelocity projectileSpeed = shooterAngularVelocityToLinearVelocity(shooterVelocity);

		// gravity in inches/sec^2
		double g = 386.09;
		LinearVelocity yComponent =
				projectileSpeed.times(
						Math.sin(
								ShooterTurretConstants.ShooterConstants.shooterAngleWithHorizontal.in(Radians)));
		double yComponentInches = yComponent.in(InchesPerSecond);
		double heightDiffInches =
				FieldConstants.hubEntranceHeight.in(Inches)
						- ShooterTurretConstants.ShooterConstants.shooterHeight.in(Inches);

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
