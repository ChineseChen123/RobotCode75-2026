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
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.RobotConstants.loopTimeSecs;
import static frc.robot.Constants.ShooterTurretConstants.ShooterConstants.*;
import static frc.robot.Constants.ShooterTurretConstants.TurretConstants.turretPositionOffset;
import static frc.robot.Constants.ShooterTurretConstants.TurretConstants.turretSoftRange;
import static frc.robot.Constants.ShooterTurretConstants.additionalPhaseDelayShooterSpeeds;
import static frc.robot.Constants.ShooterTurretConstants.phaseDelay;
import static frc.robot.Constants.ShooterTurretConstants.useVirtualTarget;
import static frc.robot.Constants.ShooterTurretConstants.virtualTargetSolveIterations;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.state.RobotStates;

public class ShooterPhysics {

	// ── Result containers ────────────────────────────────────────────────────────

	public static class TurretSetpoint {
		public Angle turretAngle;
		public AngularVelocity turretVelocity;

		public TurretSetpoint(Angle turretAngle, AngularVelocity turretVelocity) {
			this.turretAngle = turretAngle;
			this.turretVelocity = turretVelocity;
		}
	}

	// ── Timing / filtering constants ─────────────────────────────────────────────

	private static final LinearFilter turretAngleFilter =
			LinearFilter.movingAverage((int) (0.1 / loopTimeSecs));
	private static Angle lastTurretAngle = null;

	// ── Shooter speed / distance conversions ─────────────────────────────────────

	public static AngularVelocity distanceToWheelAngularVelocity(double distanceToTarget) {
		if (distanceToTarget < minShootingDistance.in(Meters)) {
			// Minimum speed needed to clear the target at short range.
			return minShootingAngularVelocity;
		}

		return RPM.of(
				Math.min(
						shooterRegressionA * distanceToTarget + shooterRegressionB,
						maxShootingAngularVelocity.in(RPM)));
	}

	public static Distance wheelAngularVelocityToDistance(AngularVelocity velocity) {
		double rpm = velocity.in(RPM);

		// Handle minimum-speed clamp.
		if (rpm <= minShootingAngularVelocity.in(RPM)) {
			return minShootingDistance;
		}

		// Handle max-speed clamp.
		rpm = Math.min(rpm, maxShootingAngularVelocity.in(RPM));

		return Meters.of((rpm - shooterRegressionB) / shooterRegressionA);
	}

	public static AngularVelocity calculateShooterSpeed(Pose2d robotPose, Pose2d targetPose) {
		Translation2d turretTranslation = getTurretPose(robotPose).getTranslation();
		double distanceToTarget = turretTranslation.getDistance(targetPose.getTranslation());
		return distanceToWheelAngularVelocity(distanceToTarget);
	}

	// ── Projectile speed models ──────────────────────────────────────────────────

	public static LinearVelocity wheelAngularVelocityToLinearVelocityRPMBased(
			AngularVelocity shooterVelocity) {
		// Recalculate launch speed from wheel speed and inertia model.
		double totalMOI = flywheelMOI + shooterWheelMOI; // in^2 / lbs
		double shooterWheelRadiusInches = shooterWheelDiameter.in(Inches) / 2.0;

		LinearVelocity surfaceWheelSpeed =
				InchesPerSecond.of(shooterVelocity.in(RadiansPerSecond) * shooterWheelRadiusInches);

		// https://www.reca.lc/flywheel
		double speedTransferPercentage =
				(20 * totalMOI)
						/ (7 * ballWeight * shooterWheelRadiusInches * shooterWheelRadiusInches / 2.0
								+ 40 * totalMOI);

		return surfaceWheelSpeed.times(speedTransferPercentage);
	}

	public static LinearVelocity wheelAngularVelocityToLinearVelocityDistanceBased(
			AngularVelocity shooterVelocity, Distance heightOfTarget) {
		// Recalculate launch speed from empirical distance-to-RPM relationship.
		double horizontalDistanceMeters = wheelAngularVelocityToDistance(shooterVelocity).in(Meters);
		double heightDiffMeters = heightOfTarget.in(Meters) - shooterHeight.in(Meters);

		double launchAngleRad = shooterAngleWithVertical.in(Radians);
		double sin = Math.sin(launchAngleRad);
		double tan = Math.tan(launchAngleRad);

		double velocity =
				Math.sqrt(
						9.8
								* horizontalDistanceMeters
								* horizontalDistanceMeters
								/ (2 * sin * sin * (horizontalDistanceMeters * (1.0 / tan) - heightDiffMeters)));

		return MetersPerSecond.of(velocity);
	}

	// ── Time of flight / virtual target ──────────────────────────────────────────

	public static Time calculateTimeToScore(Pose2d turretPose, Pose3d targetPose) {
		double distanceToTarget =
				turretPose.getTranslation().getDistance(targetPose.toPose2d().getTranslation());

		AngularVelocity shooterVelocity = distanceToWheelAngularVelocity(distanceToTarget);

		// TODO: Compare RPMBased vs DistanceBased and keep the better one.
		LinearVelocity projectileSpeed =
				wheelAngularVelocityToLinearVelocityDistanceBased(
						shooterVelocity, targetPose.getMeasureZ());

		// Gravity in inches / sec^2.
		double g = 386.09;

		LinearVelocity yComponent =
				projectileSpeed.times(Math.sin(shooterAngleWithVertical.in(Radians)));
		double yComponentInchesPerSecond = yComponent.in(InchesPerSecond);
		double heightDiffInches = targetPose.getMeasureZ().in(Inches) - shooterHeight.in(Inches);

		double secondsToScore =
				(yComponentInchesPerSecond
								+ Math.sqrt(
										yComponentInchesPerSecond * yComponentInchesPerSecond
												+ 2 * g * heightDiffInches))
						/ g;

		return Seconds.of(secondsToScore);
	}

	public static Pose2d getVirtualTarget(
			Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, int iterations) {
		Pose3d virtualTargetPose = PeddieBounds.getShootingTargetPose(robotPose);

		// Compensate for system delay by projecting robot motion forward.
		ChassisSpeeds robotRelativeSpeeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, robotPose.getRotation());

		robotPose =
				robotPose.exp(
						new Twist2d(
								robotRelativeSpeeds.vxMetersPerSecond * phaseDelay,
								robotRelativeSpeeds.vyMetersPerSecond * phaseDelay,
								robotRelativeSpeeds.omegaRadiansPerSecond * phaseDelay));

		// Re-express target-relative motion and ignore lateral/rotational target motion.
		Rotation2d targetFrame = virtualTargetPose.toPose2d().minus(robotPose).getRotation();
		ChassisSpeeds targetRelativeSpeeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, targetFrame);

		targetRelativeSpeeds.vyMetersPerSecond = 0;
		targetRelativeSpeeds.omegaRadiansPerSecond = 0;

		ChassisSpeeds robotRelativeTargetRelativeSpeeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(
						ChassisSpeeds.fromRobotRelativeSpeeds(targetRelativeSpeeds, targetFrame),
						robotPose.getRotation());

		robotPose =
				robotPose.exp(
						new Twist2d(
								robotRelativeTargetRelativeSpeeds.vxMetersPerSecond
										* additionalPhaseDelayShooterSpeeds,
								robotRelativeTargetRelativeSpeeds.vyMetersPerSecond
										* additionalPhaseDelayShooterSpeeds,
								0));

		Pose2d turretPose = getTurretPose(robotPose);

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
					PeddieBounds.getShootingTargetPose(robotPose)
							.plus(new Transform3d(new Transform2d(targetTranslation, Rotation2d.kZero)));
		}

		return virtualTargetPose.toPose2d();
	}

	// ── Geometry / angle helpers ─────────────────────────────────────────────────

	private static Pose2d getTurretPose(Pose2d robotPose) {
		return robotPose.transformBy(turretPositionOffset);
	}

	private static Pose2d getTargetPose(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
		return useVirtualTarget
				? getVirtualTarget(robotPose, fieldRelativeSpeeds, virtualTargetSolveIterations)
				: PeddieBounds.getShootingTargetPose(robotPose).toPose2d();
	}

	private static double wrapToSigned180(double angleDeg) {
		angleDeg += 180.0;
		angleDeg = (angleDeg < 0) ? (360.0 - Math.abs(angleDeg) % 360.0) % 360.0 : (angleDeg % 360.0);
		angleDeg -= 180.0;
		return angleDeg;
	}

	private static double constrainTurretAngleToSoftRange(double angleDeg) {
		double turnLimit = turretSoftRange.in(Degrees) / 2.0;

		if (Math.abs(angleDeg) <= turnLimit) {
			return angleDeg;
		}

		// Interpolate the blind spot onto the allowed side so the turret chooses
		// the legal direction instead of commanding through the deadzone.
		if (angleDeg > turnLimit) {
			// Map [turnLimit, 180] -> [turnLimit, 0]
			double t = (angleDeg - turnLimit) / (180.0 - turnLimit);
			return turnLimit * (1.0 - t);
		}

		// Map [-180, -turnLimit] -> [0, -turnLimit]
		double t = (angleDeg + 180.0) / (180.0 - turnLimit);
		return -turnLimit * t;
	}

	// ── Turret targeting ─────────────────────────────────────────────────────────

	public static TurretSetpoint calculateTurretSetpoint(
			Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
		Translation2d turretTranslation = getTurretPose(robotPose).getTranslation();
		Pose2d targetPose = getTargetPose(robotPose, fieldRelativeSpeeds);

		Rotation2d fieldRelativeToTarget =
				new Rotation2d(
						targetPose.getTranslation().getX() - turretTranslation.getX(),
						targetPose.getTranslation().getY() - turretTranslation.getY());

		// Convert field-relative heading to robot-relative turret heading.
		Angle turretAngle =
				fieldRelativeToTarget.getMeasure().minus(RobotStates.robotHeading.get().getMeasure());

		double wrappedAngleDeg = wrapToSigned180(turretAngle.in(Degrees));
		double constrainedAngleDeg = constrainTurretAngleToSoftRange(wrappedAngleDeg);

		turretAngle = Degrees.of(constrainedAngleDeg);

		if (lastTurretAngle == null) {
			lastTurretAngle = turretAngle;
		}

		AngularVelocity turretVelocity =
				DegreesPerSecond.of(
						turretAngleFilter.calculate(
								turretAngle.minus(lastTurretAngle).in(Degrees) / loopTimeSecs));

		lastTurretAngle = turretAngle;

		return new TurretSetpoint(turretAngle, turretVelocity);
	}

	public static boolean isTurretInDeadzone(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
		Translation2d turretTranslation = getTurretPose(robotPose).getTranslation();
		Pose2d targetPose = getTargetPose(robotPose, fieldRelativeSpeeds);

		Rotation2d fieldRelativeToTarget =
				new Rotation2d(
						targetPose.getTranslation().getX() - turretTranslation.getX(),
						targetPose.getTranslation().getY() - turretTranslation.getY());

		Angle turretAngle =
				fieldRelativeToTarget.getMeasure().minus(RobotStates.robotHeading.get().getMeasure());

		double wrappedAngleDeg = wrapToSigned180(turretAngle.in(Degrees));
		return Math.abs(wrappedAngleDeg) > turretSoftRange.in(Degrees) / 2.0;
	}
}
