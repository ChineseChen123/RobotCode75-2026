package frc.lib.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.RobotConstants.loopTimeSecs;
import static frc.robot.Constants.ShooterTurretConstants.ShooterConstants.*;
import static frc.robot.Constants.ShooterTurretConstants.TurretConstants.turretPositionOffset;
import static frc.robot.Constants.ShooterTurretConstants.TurretConstants.turretSoftRange;
import static frc.robot.Constants.ShooterTurretConstants.useVirtualTarget;
import static frc.robot.Constants.ShooterTurretConstants.virtualTargetSolveIterations;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.FieldConstants;
import frc.robot.state.RobotStates;

public class ShooterPhysics {

	public static class TurretSetpoint {
		public Angle turretAngle;
		public AngularVelocity turretVelocity;

		public TurretSetpoint(Angle turretAngle, AngularVelocity turretVelocity) {
			this.turretAngle = turretAngle;
			this.turretVelocity = turretVelocity;
		}
	}

	public static AngularVelocity distanceToWheelAngularVelocity(double distanceToHub) {
		if (distanceToHub < 2.263119) { // min speed to pass top of hub
			return RPM.of(2430);
		}
		return RPM.of(Math.min((shooterRegressionA * distanceToHub + shooterRegressionB), 3800));
	}

	public static Distance wheelAngularVelocityToDistance(AngularVelocity velocity) {
		double rpm = velocity.in(RotationsPerSecond) * 60.0;

		// Handle capped minimum case
		if (rpm <= 2430) {
			return Meters.of(2.263119);
		}

		// Handle 3800 RPM cap
		rpm = Math.min(rpm, 3800);

		return Meters.of((rpm - shooterRegressionB) / shooterRegressionA);
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

	private static final double phaseDelay = 0.05;
	private static final double additionalPhaseDelayShooterSpeeds = 0.03;

	public static Time calculateTimeToScore(Pose2d turretPose, Pose2d targetHubPose) {

		AngularVelocity shooterVelocity =
				distanceToWheelAngularVelocity(
						turretPose.getTranslation().getDistance(targetHubPose.getTranslation()));

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

		ChassisSpeeds robotRelativeSpeeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, robotPose.getRotation());

		robotPose =
				robotPose.exp(
						new Twist2d(
								robotRelativeSpeeds.vxMetersPerSecond * phaseDelay,
								robotRelativeSpeeds.vyMetersPerSecond * phaseDelay,
								robotRelativeSpeeds.omegaRadiansPerSecond * phaseDelay));

		ChassisSpeeds hubRelativeSpeeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(
						fieldRelativeSpeeds, virtualTargetPose.minus(robotPose).getRotation());

		// System.out.println(hubRelativeSpeeds.vxMetersPerSecond);

		hubRelativeSpeeds.vyMetersPerSecond = 0;
		hubRelativeSpeeds.omegaRadiansPerSecond = 0;

		ChassisSpeeds robotRelativeHubRelativeSpeeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(
						ChassisSpeeds.fromRobotRelativeSpeeds(
								hubRelativeSpeeds, virtualTargetPose.minus(robotPose).getRotation()),
						robotPose.getRotation());

		robotPose =
				robotPose.exp(
						new Twist2d(
								robotRelativeHubRelativeSpeeds.vxMetersPerSecond
										* additionalPhaseDelayShooterSpeeds,
								robotRelativeHubRelativeSpeeds.vyMetersPerSecond
										* additionalPhaseDelayShooterSpeeds,
								0));

		Pose2d turretPose = robotPose.transformBy(turretPositionOffset);

		Rotation2d robotAngle = robotPose.getRotation();
		double turretVelocityX =
				fieldRelativeSpeeds.vxMetersPerSecond
						- fieldRelativeSpeeds.omegaRadiansPerSecond
								* (turretPositionOffset.getY() * robotAngle.getCos()
										+ turretPositionOffset.getX() * robotAngle.getSin());
		double turretVelocityY =
				fieldRelativeSpeeds.vyMetersPerSecond
						+ fieldRelativeSpeeds.omegaRadiansPerSecond
								* (turretPositionOffset.getX() * robotAngle.getCos()
										- turretPositionOffset.getY() * robotAngle.getSin());

		for (int i = 0; i < iterations; i++) {
			Time tofEstimate = calculateTimeToScore(turretPose, virtualTargetPose);

			Translation2d targetTranslation =
					new Translation2d(
							MetersPerSecond.of(-turretVelocityX).times(tofEstimate),
							MetersPerSecond.of(-turretVelocityY).times(tofEstimate));
			virtualTargetPose =
					PeddieBounds.getHubTarget().plus(new Transform2d(targetTranslation, Rotation2d.kZero));
		}
		return virtualTargetPose;
	}

	private static LinearFilter turretAngleFilter =
			LinearFilter.movingAverage((int) (0.1 / loopTimeSecs));
	private static Angle lastTurretAngle = null;

	public static TurretSetpoint calculateTurretSetpoint(
			Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
		Translation2d turretPose = robotPose.transformBy(turretPositionOffset).getTranslation();
		Pose2d targetHubPose =
				useVirtualTarget
						? getVirtualTarget(robotPose, fieldRelativeSpeeds, virtualTargetSolveIterations)
						: PeddieBounds.getHubTarget();

		Rotation2d fieldRelativeToHub =
				new Rotation2d(
						targetHubPose.getTranslation().getX() - turretPose.getX(),
						targetHubPose.getTranslation().getY() - turretPose.getY());

		// robot relative
		Angle turretAngle =
				fieldRelativeToHub.getMeasure().minus(RobotStates.robotHeading.get().getMeasure());

		double angleDeg = turretAngle.in(Degrees);
		// Wrap angle to [-180, 180)
		angleDeg += 180;
		angleDeg = (angleDeg < 0) ? (360 - Math.abs(angleDeg) % 360) % 360 : (angleDeg % 360);
		angleDeg -= 180;

		// Interpolate blind spot in opposite direction by factor of 3
		if (Math.abs(angleDeg) > turretSoftRange.in(Degrees) / 2.0) {
			double turnLimit = turretSoftRange.in(Degrees) / 2.0;
			// Map [135, 180] -> [135, 0] linearly
			if (angleDeg > turnLimit) { // (135, 180]
				double t = (angleDeg - turnLimit) / (180 - turnLimit); // 0..1
				angleDeg = turnLimit * (1.0 - t); // 135..0
			} else {
				// Map [-180, -135] -> [0, -135] linearly
				// angleDeg in [-180, -135)
				double t = (angleDeg + 180.0) / (180 - turnLimit); // 0..1
				angleDeg = -turnLimit * t;
			}
		}

		turretAngle = Degrees.of(angleDeg);

		if (lastTurretAngle == null) lastTurretAngle = turretAngle;

		AngularVelocity turretVelocity =
				DegreesPerSecond.of(
						turretAngleFilter.calculate(
								turretAngle.minus(lastTurretAngle).in(Degrees) / loopTimeSecs));

		lastTurretAngle = turretAngle;

		return new TurretSetpoint(turretAngle, turretVelocity);
	}

	public static boolean isTurretInDeadzone(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
		Translation2d turretPose = robotPose.transformBy(turretPositionOffset).getTranslation();
		Pose2d targetHubPose =
				useVirtualTarget
						? getVirtualTarget(robotPose, fieldRelativeSpeeds, virtualTargetSolveIterations)
						: PeddieBounds.getHubTarget();

		Rotation2d fieldRelativeToHub =
				new Rotation2d(
						targetHubPose.getTranslation().getX() - turretPose.getX(),
						targetHubPose.getTranslation().getY() - turretPose.getY());

		// robot relative
		Angle turretAngle =
				fieldRelativeToHub.getMeasure().minus(RobotStates.robotHeading.get().getMeasure());

		double angleDeg = turretAngle.in(Degrees);
		// Wrap angle to [-180, 180)
		angleDeg += 180;
		angleDeg = (angleDeg < 0) ? (360 - Math.abs(angleDeg) % 360) % 360 : (angleDeg % 360);
		angleDeg -= 180;

		return (Math.abs(angleDeg) > turretSoftRange.in(Degrees) / 2.0);
	}
}
