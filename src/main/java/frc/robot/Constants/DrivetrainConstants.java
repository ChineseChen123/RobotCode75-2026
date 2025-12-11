// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

/** Swerve drive constants */
public final class DrivetrainConstants {
	/** Drivetrain CANBus name */
	public static final String driveBusName = "Drivetrain";

	public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

	/* Drivetrain Constants */
	public static final Distance trackLength = Inches.of(22.75);
	public static final Distance trackWidth = Inches.of(22.75);

	public static final Distance wheelDiameter = Inches.of(4);
	public static final Distance wheelCircumference = Meters.of(wheelDiameter.in(Meters) * Math.PI);

	public static final class ControllerConstants {
		public static final double toleranceRadians = Units.degreesToRadians(1.5);
		public static final double toleranceTranslation = .01;

		public static final class RotationAlign {

			public static final double kp = 2.2;
			public static final double kd = 0;
			public static final double maxVelocityMultiplier = 0.8;
			public static final double maxAccelerationMultiplier = 0.8;
			public static final double loopPeriodSeconds = 0.02;
		}

		public static final class OdometryAlign {
			public static final double xP = 2;
			public static final double xI = 0.0;
			public static final double xD = 0.0;

			public static final double tP = 3;
			public static final double tI = 0.0;
			public static final double tD = 0.0;
		}

		public static final class VisionAlign {
			public static final double xP = 0.0;
			public static final double xI = 0.0;
			public static final double xD = 0.0;

			public static final double yP = 0.7;
			public static final double yI = 0.0;
			public static final double yD = 0.0;
		}

		public static final LinearVelocity maxVelocity = MetersPerSecond.of(1);
		public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(1);

		/** Radians per Second */
		public static final AngularVelocity maxAngularVelocity =
				RadiansPerSecond.of(
						maxSpeed.in(MetersPerSecond)
								/ Math.hypot(trackWidth.in(Meters) / 2.0, trackLength.in(Meters) / 2.0));

		/** Radians per Second per Second */
		public static final AngularAcceleration maxAngularAcceleration =
				RadiansPerSecondPerSecond.of(
						maxSpeed.in(MetersPerSecond)
								/ Math.hypot(trackWidth.in(Meters) / 2.0, trackLength.in(Meters) / 2.0));

		public static final AngularVelocity maxAngularVelocityAuto = maxAngularVelocity.div(2);
		public static final AngularAcceleration maxAngularAccelerationAuto =
				maxAngularAcceleration.div(2);
	}

	/*
	 * Swerve Kinematics
	 * No need to ever change this unless you are not doing a traditional
	 * rectangular/square 4 module swerve
	 */
	public static final SwerveDriveKinematics swerveKinematics =
			new SwerveDriveKinematics(
					new Translation2d(
							DrivetrainConstants.trackWidth.in(Meters) / 2.0,
							DrivetrainConstants.trackLength.in(Meters) / 2.0),
					new Translation2d(
							DrivetrainConstants.trackWidth.in(Meters) / 2.0,
							-DrivetrainConstants.trackLength.in(Meters) / 2.0),
					new Translation2d(
							-DrivetrainConstants.trackWidth.in(Meters) / 2.0,
							DrivetrainConstants.trackLength.in(Meters) / 2.0),
					new Translation2d(
							-DrivetrainConstants.trackWidth.in(Meters) / 2.0,
							-DrivetrainConstants.trackLength.in(Meters) / 2.0));

	/* Module Gear Ratios */
	// ratio of motor turns to mechanism turns
	public static final double driveGearRatio = 6.75; // L2
	public static final double angleGearRatio = 150.0 / 7.0; // ~21:1 ratio

	/** Meters per Second */
	public static final LinearVelocity maxSpeed = MetersPerSecond.of(3.5);

	public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(3);

	/** Radians per Second */
	public static final AngularVelocity maxAngularVelocity =
			RadiansPerSecond.of(
					maxSpeed.in(MetersPerSecond)
							/ Math.hypot(trackLength.in(Meters) / 2.0, trackWidth.in(Meters) / 2.0));

	/** Radians per Second per Second */
	public static final AngularAcceleration maxAngularAcceleration =
			RadiansPerSecondPerSecond.of(
					maxSpeed.in(MetersPerSecond)
							/ Math.hypot(trackLength.in(Meters) / 2.0, trackWidth.in(Meters) / 2.0));
}
