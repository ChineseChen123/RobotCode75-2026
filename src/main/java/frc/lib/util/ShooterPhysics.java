package frc.lib.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterPhysics {
	// basically just lookup table
	public static class CompensatedShot {
		public final Angle turretAngle; // field-relative yaw to aim turret
		public final AngularVelocity shooterSpeed; // compensated shooter wheel speed

		public CompensatedShot(Angle turretAngle, AngularVelocity shooterSpeed) {
			this.turretAngle = turretAngle;
			this.shooterSpeed = shooterSpeed;
		}
	}

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
				ShooterConstants.Shooter.flywheelMOI
						+ ShooterConstants.Shooter.shooterWheelMOI; // in^2 / lbs
		double shooterWheelRadiusInches =
				(ShooterConstants.Shooter.shooterWheelDiameter.in(Inches) / 2);
		LinearVelocity surfaceWheelSpeed =
				InchesPerSecond.of(shooterVelocity.in(RadiansPerSecond) * shooterWheelRadiusInches);

		// https://www.reca.lc/flywheel
		double speedTransferPercentage =
				(20 * totalMOI)
						/ (7
										* ShooterConstants.Shooter.ballWeight
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
				ShooterConstants.Shooter.flywheelMOI
						+ ShooterConstants.Shooter.shooterWheelMOI; // in^2 / lbs
		double shooterWheelRadiusInches =
				(ShooterConstants.Shooter.shooterWheelDiameter.in(Inches) / 2);
		double speedTransferPercentage =
				(20 * totalMOI)
						/ (7
										* ShooterConstants.Shooter.ballWeight
										* shooterWheelRadiusInches
										* shooterWheelRadiusInches
										/ 2
								+ 40 * totalMOI);

		double surfaceSpeedInchesPerSecond =
				projectileSpeed.in(InchesPerSecond) / speedTransferPercentage;

		double angularVelocityRadPerSec = surfaceSpeedInchesPerSecond / shooterWheelRadiusInches;

		return RadiansPerSecond.of(angularVelocityRadPerSec);
	}

	public static Time calculateTimeToScore(Pose2d robotPose, Pose2d targetHubPose) {

		AngularVelocity shooterVelocity =
				distanceToAngularVelocity(
						robotPose.getTranslation().getDistance(targetHubPose.getTranslation()));
		LinearVelocity projectileSpeed = shooterAngularVelocityToLinearVelocity(shooterVelocity);

		// gravity in inches/sec^2
		double g = 386.09;
		LinearVelocity yComponent =
				projectileSpeed.times(
						Math.sin(ShooterConstants.Shooter.shooterAngleWithHorizontal.in(Radians)));
		double yComponentInches = yComponent.in(InchesPerSecond);
		double heightDiffInches =
				FieldConstants.hubEntranceHeight.in(Inches)
						- ShooterConstants.Shooter.shooterHeight.in(Inches);

		double secondsToScore =
				(yComponentInches
								+ Math.sqrt(yComponentInches * yComponentInches - 2 * g * (heightDiffInches)))
						/ g;

		return Seconds.of(secondsToScore);
	}

	public static AngularVelocity calculateCompensatedAngularVelocity(
			Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
		return calculateCompensatedShot(robotPose, fieldRelativeSpeeds).shooterSpeed;
	}

	public static Angle calculateCompensatedTurretAngle(
			Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
		return calculateCompensatedShot(robotPose, fieldRelativeSpeeds).turretAngle;
	}

	public static CompensatedShot calculateCompensatedShot(
			Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {

		// ------------------------------------------------------------
		// 1. Determine which hub to target (same logic you already use)
		// ------------------------------------------------------------
		Pose2d targetHubPose =
				DriverStation.getAlliance().isPresent()
								&& DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
						? FieldConstants.blueHub
						: FieldConstants.redHub;

		Translation2d robotTranslation = robotPose.getTranslation();
		Translation2d hubTranslation = targetHubPose.getTranslation();

		// Horizontal displacement vector from robot to hub
		double dxMeters = hubTranslation.getX() - robotTranslation.getX();
		double dyMeters = hubTranslation.getY() - robotTranslation.getY();

		double horizontalDistanceMeters = Math.hypot(dxMeters, dyMeters);

		// Unit vector toward hub in field coordinates
		double ux = dxMeters / horizontalDistanceMeters;
		double uy = dyMeters / horizontalDistanceMeters;

		// ------------------------------------------------------------
		// 2. Estimate time of flight using existing stationary physics
		//    This gives us a reasonable baseline without needing a
		//    full iterative solver.
		// ------------------------------------------------------------

		// TODO: THIS IS A MAJOR ESTIMATE. TECHNICALLY THIS ISNT A FULL SOLVE
		// BASICALLY THIS TIME IS IF WE WERE STATIONARY!!
		// prob should change this...
		Time tofEstimate = calculateTimeToScore(robotPose, targetHubPose);
		double t = tofEstimate.in(Seconds);

		// ------------------------------------------------------------
		// 3. Compute required WORLD-FRAME projectile velocity
		//    to reach the hub in time t
		//
		// Horizontal:
		//   v_world_xy = distance / t
		//
		// Vertical:
		//   v_world_z = (Δh + 0.5 g t²) / t
		// ------------------------------------------------------------
		double distanceInches = Inches.of(horizontalDistanceMeters).in(Inches);
		double vWorldHorizontalInchesPerSec = distanceInches / t;

		// Height difference
		double heightDiffInches =
				FieldConstants.hubEntranceHeight.in(Inches)
						- ShooterConstants.Shooter.shooterHeight.in(Inches);

		// gravity in inches/sec^2
		double g = 386.09;

		double vWorldZ = (heightDiffInches + 0.5 * g * t * t) / t;

		// Horizontal world velocity vector toward hub
		double vWorldX = vWorldHorizontalInchesPerSec * ux;
		double vWorldY = vWorldHorizontalInchesPerSec * uy;

		// ------------------------------------------------------------
		// 4. Subtract robot velocity to get REQUIRED RELATIVE velocity
		//
		// v_rel = v_world - v_robot
		// ------------------------------------------------------------
		double robotVxInches =
				MetersPerSecond.of(fieldRelativeSpeeds.vxMetersPerSecond).in(InchesPerSecond);
		double robotVyInches =
				MetersPerSecond.of(fieldRelativeSpeeds.vyMetersPerSecond).in(InchesPerSecond);

		double vRelX = vWorldX - robotVxInches;
		double vRelY = vWorldY - robotVyInches;
		double vRelZ = vWorldZ;

		// ------------------------------------------------------------
		// 5. Compute required relative exit speed magnitude
		// ------------------------------------------------------------
		double requiredExitSpeedIPS = Math.sqrt(vRelX * vRelX + vRelY * vRelY + vRelZ * vRelZ);
		LinearVelocity requiredExitSpeed = InchesPerSecond.of(requiredExitSpeedIPS);

		// Convert exit speed → angular velocity
		// (You will need to implement this mapping)
		AngularVelocity compensatedShooterSpeed =
				linearVelocityToShooterAngularVelocity(requiredExitSpeed);

		Angle turretYaw = Radians.of(Math.atan2(vRelY, vRelX));

		return new CompensatedShot(turretYaw, compensatedShooterSpeed);
	}

	public static Pose2d getVirtualTarget(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, int iterations) {

		Pose2d virtualTargetPose =
				DriverStation.getAlliance().isPresent()
								&& DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
						? FieldConstants.blueHub
						: FieldConstants.redHub;
        
        for (int i=0; i<iterations; i++) {
            Time tofEstimate = calculateTimeToScore(robotPose, virtualTargetPose);

            Translation2d targetTranslation =
                    new Translation2d(
                            MetersPerSecond.of(-fieldRelativeSpeeds.vxMetersPerSecond).times(tofEstimate),
                            MetersPerSecond.of(-fieldRelativeSpeeds.vyMetersPerSecond).times(tofEstimate));
            virtualTargetPose =
                    virtualTargetPose.plus(new Transform2d(targetTranslation, Rotation2d.kZero));
        }
		return virtualTargetPose;
	}
}
