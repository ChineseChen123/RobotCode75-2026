// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

/** Swerve drive constants */
public final class DrivetrainConstants {
	/* Drivetrain Constants */

	public static final class ControllerConstants {
		// tolerances for chezy controller
		public static final double toleranceRadians = Units.degreesToRadians(1.5);
		public static final double toleranceTranslation = .01;

		// rotation and chezy controllers
		public static final class RotationAlign {
			public static final double kp = 2.2;
			public static final double kd = 0;
			public static final double maxVelocityMultiplier = 0.8;
			public static final double maxAccelerationMultiplier = 0.8;
			public static final double loopPeriodSeconds = 0.02;
		}

		// chezy controller only
		public static final class OdometryAlign {
			public static final double xP = 2;
			public static final double xI = 0.0;
			public static final double xD = 0.0;

			public static final double tP = 3;
			public static final double tI = 0.0;
			public static final double tD = 0.0;
		}

		// only applicable when robot is being controlled by a PID controller
		public static final LinearVelocity maxVelocity = MetersPerSecond.of(1);
		public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(1);

		// radians per second
		public static final AngularVelocity maxAngularVelocity =
				RadiansPerSecond.of(
						maxVelocity.in(MetersPerSecond)
								/ Math.hypot(trackWidth.in(Meters) / 2.0, trackLength.in(Meters) / 2.0));

		// radians per second per second
		public static final AngularAcceleration maxAngularAcceleration =
				RadiansPerSecondPerSecond.of(
						maxVelocity.in(MetersPerSecond)
								/ Math.hypot(trackWidth.in(Meters) / 2.0, trackLength.in(Meters) / 2.0));

		public static final AngularVelocity maxAngularVelocityAuto = maxAngularVelocity.div(2);
		public static final AngularAcceleration maxAngularAccelerationAuto =
				maxAngularAcceleration.div(2);
	}

	// ── Phoenix Swerve API Constants ────────────────────────────────────────────────

	// Both sets of gains need to be tuned to your individual robot.

	// The steer motor uses any SwerveModule.SteerRequestType control request with the
	// output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
	private static final Slot0Configs steerGains =
			new Slot0Configs()
					.withKP(50)
					.withKI(0)
					.withKD(0.5)
					.withKS(0.05)
					.withKV(0)
					.withKA(0)
					.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
	// When using closed-loop control, the drive motor uses the control
	// output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
	private static final Slot0Configs driveGains =
			new Slot0Configs().withKP(0.2).withKI(0).withKD(0).withKS(0).withKV(0.12);

	// The closed-loop output type to use for the steer motors;
	// This affects the PID/FF gains for the steer motors
	private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
	// The closed-loop output type to use for the drive motors;
	// This affects the PID/FF gains for the drive motors
	private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

	// The type of motor used for the drive motor
	private static final DriveMotorArrangement kDriveMotorType =
			DriveMotorArrangement.TalonFX_Integrated;
	// The type of motor used for the drive motor
	private static final SteerMotorArrangement kSteerMotorType =
			SteerMotorArrangement.TalonFX_Integrated;

	// The remote sensor feedback type to use for the steer motors;
	// When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
	private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

	// The stator current at which the wheels start to slip;
	// This needs to be tuned to your individual robot
	private static final Current kSlipCurrent = Amps.of(120.0);

	// Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
	// Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
	private static final TalonFXConfiguration driveInitialConfigs =
			new TalonFXConfiguration()
					.withCurrentLimits(
							new CurrentLimitsConfigs()
									.withSupplyCurrentLimit(Amps.of(40))
									.withSupplyCurrentLimitEnable(true));
	private static final TalonFXConfiguration steerInitialConfigs =
			new TalonFXConfiguration()
					.withCurrentLimits(
							new CurrentLimitsConfigs()
									// Swerve azimuth does not require much torque output, so we can set a relatively
									// low
									// stator current limit to help avoid brownouts without impacting performance.
									.withSupplyCurrentLimit(Amps.of(40))
									.withSupplyCurrentLimitEnable(true)
									.withStatorCurrentLimit(Amps.of(60))
									.withStatorCurrentLimitEnable(true));
	private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
	// Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
	private static final Pigeon2Configuration pigeonConfigs = null;

	// CAN bus that the devices are located on;
	// All swerve devices must share the same CAN bus
	public static final CANBus kCANBus = new CANBus("drivetrain", "./logs/example.hoot");

	// Theoretical free speed (m/s) at 12 V applied output;
	// This needs to be tuned to your individual robot
	public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(3);

	// Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
	// This may need to be tuned to your individual robot
	private static final double kCoupleRatio = 3.857142857142857;

	private static final double kDriveGearRatio = 6.026785714285714;
	private static final double kSteerGearRatio = 26.09090909090909;

	public static final Distance trackLength = Inches.of(22.1875);
	public static final Distance trackWidth = Inches.of(22.1875);
	private static final Distance kWheelRadius = Inches.of(2);

	private static final boolean kInvertLeftSide = false;
	private static final boolean kInvertRightSide = true;

	private static final int kPigeonId = 19;

	// These are only used for simulation
	private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
	private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
	// Simulated voltage necessary to overcome friction
	private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
	private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

	// meters per second
	public static final LinearVelocity maxVelocity = MetersPerSecond.of(3.5);

	public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(3);

	public static final double speedClampMultiplier = 0.25;

	// radians per second
	public static final AngularVelocity maxAngularVelocity =
			RadiansPerSecond.of(
					maxVelocity.in(MetersPerSecond)
							/ Math.hypot(trackLength.in(Meters) / 2.0, trackWidth.in(Meters) / 2.0));

	// radians per second per second
	public static final AngularAcceleration maxAngularAcceleration =
			RadiansPerSecondPerSecond.of(
					maxVelocity.in(MetersPerSecond)
							/ Math.hypot(trackLength.in(Meters) / 2.0, trackWidth.in(Meters) / 2.0));

	public static final SwerveDrivetrainConstants SwerveDrivetrainConstants =
			new SwerveDrivetrainConstants()
					.withCANBusName(kCANBus.getName())
					.withPigeon2Id(kPigeonId)
					.withPigeon2Configs(pigeonConfigs);

	private static final SwerveModuleConstantsFactory<
					TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
			ConstantCreator =
					new SwerveModuleConstantsFactory<
									TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
							.withDriveMotorGearRatio(kDriveGearRatio)
							.withSteerMotorGearRatio(kSteerGearRatio)
							.withCouplingGearRatio(kCoupleRatio)
							.withWheelRadius(kWheelRadius)
							.withSteerMotorGains(steerGains)
							.withDriveMotorGains(driveGains)
							.withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
							.withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
							.withSlipCurrent(kSlipCurrent)
							.withSpeedAt12Volts(kSpeedAt12Volts)
							.withDriveMotorType(kDriveMotorType)
							.withSteerMotorType(kSteerMotorType)
							.withFeedbackSource(kSteerFeedbackType)
							.withDriveMotorInitialConfigs(driveInitialConfigs)
							.withSteerMotorInitialConfigs(steerInitialConfigs)
							.withEncoderInitialConfigs(encoderInitialConfigs)
							.withSteerInertia(kSteerInertia)
							.withDriveInertia(kDriveInertia)
							.withSteerFrictionVoltage(kSteerFrictionVoltage)
							.withDriveFrictionVoltage(kDriveFrictionVoltage);

	// Front Left
	private static final int kFrontLeftDriveMotorId = 12;
	private static final int kFrontLeftSteerMotorId = 22;
	private static final int kFrontLeftEncoderId = 32;
	private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.2109375);
	private static final boolean kFrontLeftSteerMotorInverted = false;
	private static final boolean kFrontLeftEncoderInverted = false;

	private static final Distance kFrontLeftXPos = trackWidth.div(2);
	private static final Distance kFrontLeftYPos = trackLength.div(2);

	// Front Right
	private static final int kFrontRightDriveMotorId = 11;
	private static final int kFrontRightSteerMotorId = 21;
	private static final int kFrontRightEncoderId = 31;
	private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.474609375);
	private static final boolean kFrontRightSteerMotorInverted = false;
	private static final boolean kFrontRightEncoderInverted = false;

	private static final Distance kFrontRightXPos = trackWidth.div(2);
	private static final Distance kFrontRightYPos = trackLength.div(-2);

	// Back Left
	private static final int kBackLeftDriveMotorId = 14;
	private static final int kBackLeftSteerMotorId = 24;
	private static final int kBackLeftEncoderId = 34;
	private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.365966796875);
	private static final boolean kBackLeftSteerMotorInverted = false;
	private static final boolean kBackLeftEncoderInverted = false;

	private static final Distance kBackLeftXPos = trackWidth.div(-2);
	private static final Distance kBackLeftYPos = trackLength.div(2);

	// Back Right
	private static final int kBackRightDriveMotorId = 13;
	private static final int kBackRightSteerMotorId = 23;
	private static final int kBackRightEncoderId = 33;
	private static final Angle kBackRightEncoderOffset = Rotations.of(-0.212890625);
	private static final boolean kBackRightSteerMotorInverted = false;
	private static final boolean kBackRightEncoderInverted = false;

	private static final Distance kBackRightXPos = trackLength.div(-2);
	private static final Distance kBackRightYPos = trackWidth.div(-2);

	public static final SwerveModuleConstants<
					TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
			FrontLeft =
					ConstantCreator.createModuleConstants(
							kFrontLeftSteerMotorId,
							kFrontLeftDriveMotorId,
							kFrontLeftEncoderId,
							kFrontLeftEncoderOffset,
							kFrontLeftXPos,
							kFrontLeftYPos,
							kInvertLeftSide,
							kFrontLeftSteerMotorInverted,
							kFrontLeftEncoderInverted);
	public static final SwerveModuleConstants<
					TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
			FrontRight =
					ConstantCreator.createModuleConstants(
							kFrontRightSteerMotorId,
							kFrontRightDriveMotorId,
							kFrontRightEncoderId,
							kFrontRightEncoderOffset,
							kFrontRightXPos,
							kFrontRightYPos,
							kInvertRightSide,
							kFrontRightSteerMotorInverted,
							kFrontRightEncoderInverted);
	public static final SwerveModuleConstants<
					TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
			BackLeft =
					ConstantCreator.createModuleConstants(
							kBackLeftSteerMotorId,
							kBackLeftDriveMotorId,
							kBackLeftEncoderId,
							kBackLeftEncoderOffset,
							kBackLeftXPos,
							kBackLeftYPos,
							kInvertLeftSide,
							kBackLeftSteerMotorInverted,
							kBackLeftEncoderInverted);
	public static final SwerveModuleConstants<
					TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
			BackRight =
					ConstantCreator.createModuleConstants(
							kBackRightSteerMotorId,
							kBackRightDriveMotorId,
							kBackRightEncoderId,
							kBackRightEncoderOffset,
							kBackRightXPos,
							kBackRightYPos,
							kInvertRightSide,
							kBackRightSteerMotorInverted,
							kBackRightEncoderInverted);
}
