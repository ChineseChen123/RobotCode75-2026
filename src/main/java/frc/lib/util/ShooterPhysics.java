package frc.lib.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.FieldConstants;
import frc.robot.state.RobotStates;

public class ShooterPhysics {

	public static AngularVelocity distanceToWheelAngularVelocity(double distanceToHub) {
		if (distanceToHub < 2.31280203) { // min speed to pass top of hub
			return RotationsPerSecond.of(2650 / 60);
		}
		return RotationsPerSecond.of(
				Math.min(
								(shooterRegressionA * Math.sqrt(distanceToHub - shooterRegressionC)
												+ shooterRegressionB)
										, // TODO; wtf
								4000)
						/ 60.0);
	}

	public static Distance wheelAngularVelocityToDistance(AngularVelocity velocity) {
		double rpm = velocity.in(RotationsPerSecond) * 60.0;

		// Handle capped minimum case
		if (rpm <= 2650) {
			return Meters.of(2.31280203);
		}

		// Handle 4000 RPM cap
		rpm = Math.min(rpm, 4000);

		return Meters.of(
				Math.pow((rpm - shooterRegressionB) / shooterRegressionA, 2) + shooterRegressionC);
	}

	public static AngularVelocity calculateShooterSpeed(Pose2d robotPose, Pose2d targetHubPose) {
		Translation2d turretPose = robotPose.transformBy(turretPositionOffset).getTranslation();
		return distanceToWheelAngularVelocity(turretPose.getDistance(targetHubPose.getTranslation()));
	}

	public static LinearVelocity wheelAngularVelocityToLinearVelocityRPMBased(
			AngularVelocity shooterVelocity) {
		/* Recalc RPM --> Angular Velocity based on shooter geometry. NOT based on empirical data */
		double totalMOI = flywheelMOI + shooterWheelMOI; // in^2 / lbs
		double shooterWheelRadiusInches = (shooterWheelDiameter.in(Inches) / 2);
		LinearVelocity surfaceWheelSpeed =
				InchesPerSecond.of(shooterVelocity.in(RadiansPerSecond) * shooterWheelRadiusInches);

		// https://www.reca.lc/flywheel
		double speedTransferPercentage =
				(20 * totalMOI)
						/ (7 * ballWeight * shooterWheelRadiusInches * shooterWheelRadiusInches / 2
								+ 40 * totalMOI);
		LinearVelocity projectileSpeed = surfaceWheelSpeed.times(speedTransferPercentage);
		return projectileSpeed;
	}

	public static LinearVelocity wheelAngularVelocityToLinearVelocityDistanceBased(
			AngularVelocity shooterVelocity) {
		/* Recalc RPM --> Angular Velocity based on measured RPM --> Distance data. */

		// SQRT(9.8*B2*B2/(2*SIN(F$18)*SIN(F$18)*(B2*COT(F$18)-F$19)))

		double meters = wheelAngularVelocityToDistance(shooterVelocity).in(Meters);

		double heightDiffMeters =
				FieldConstants.hubEntranceHeight.in(Meters) - shooterHeight.in(Meters);

		double velocity =
				Math.sqrt(
						9.8
								* meters
								* meters
								/ (2
										* Math.sin(
												shooterAngleWithVertical.in(Radians)
														* Math.sin(shooterAngleWithVertical.in(Radians))
														* (meters * (1 / Math.tan(shooterAngleWithVertical.in(Radians)))
																- heightDiffMeters))));

		return MetersPerSecond.of(velocity);
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
				distanceToWheelAngularVelocity(turretPose.getDistance(targetHubPose.getTranslation()));
			
		// TODO: Test both RPMBased and DistanceBased and see which is better
		LinearVelocity projectileSpeed = wheelAngularVelocityToLinearVelocityRPMBased(shooterVelocity);

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
