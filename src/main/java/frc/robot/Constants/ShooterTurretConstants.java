// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ShooterTurretConstants.ShooterConstants.minShootingDistance;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.LinearInterpolationMap;

/** Add your docs here. */
public class ShooterTurretConstants {

	public static final boolean useVirtualTarget = true;
	public static final int virtualTargetSolveIterations = 12;

	public static final double phaseDelay = 0.05;
	public static final double additionalPhaseDelayShooterSpeeds = 0.05;

	public class ShooterConstants {
		// Kraken X60s
		public static final int shooterMotor1CanID = 41;
		public static final int shooterMotor2CanID = 42;

		public static final double shooterGearRatio = 1.5;

		public static final Distance shooterWheelDiameter = Inches.of(4);

		public static final AngularVelocity defaultShooterSpeed = RotationsPerSecond.of(0);
		public static final AngularVelocity reverseShooterSpeed = RotationsPerSecond.of(-10);

		public static final double shooterVelocityTolerance = 150.0; // rpm

		public static final Distance feedingDistPastBump = Meters.of(3);

		// Note that the flywheel MOI effective to the motors is multiplied by the gear ratio squared.
		// <-- from recalc
		public static final Distance shooterWheelRadius = Inches.of(2);
		public static final Mass shooterWheelIndividualWeight = Pounds.of(.67);
		public static final int numberOfShooterWheels = 2;

		public static final double flywheelMOI = 0; // in^2 / lbs
		public static final double shooterWheelMOI =
				.5
						* (shooterWheelIndividualWeight.in(Pounds) * numberOfShooterWheels)
						* (shooterWheelRadius.in(Inches)
								* shooterWheelRadius.in(Inches)); // in^2 / lbs, 1/2mr^2 approx.
		public static final double ballWeight = .474; // lbs
		public static final Angle shooterAngleWithVertical = Degrees.of(25);
		public static final Distance shooterHeight = Inches.of(19.5);

		public static final Distance minShootingDistance = Meters.of(2.058);
		public static final AngularVelocity minShootingAngularVelocity = RPM.of(2026);
		public static final AngularVelocity maxShootingAngularVelocity = RPM.of(3800);
		public static final LinearInterpolationMap distanceToRPMMap =
				new LinearInterpolationMap()
						.add(minShootingDistance.in(Meters), minShootingAngularVelocity.in(RPM))
						.add(2.418, 2150)
						.add(2.88, 2280)
						.add(3.07, 2350)
						.add(3.365, 2425)
						.add(3.855, 2550)
						.add(4.00, 2640)
						.add(4.24, 2680)
						.add(4.68, 2815)
						.add(5.26, 3000);

		public static final double velocityAdjustmentDeltaRPM = 10;

		public static final class MotorConfigs {

			public static final TalonFXConfiguration m_shooterMotorConfig = new TalonFXConfiguration();

			// Neutral modes and inverts
			public static final InvertedValue shooterMotorInverted = InvertedValue.Clockwise_Positive;

			public static final NeutralModeValue shooterMotorNeutralMode = NeutralModeValue.Coast;

			public static final Frequency timeSyncFreq = Hertz.of(250);

			public static final Time shooterMotorCurrentThresholdTime = Seconds.of(0.5);

			public static final Current shooterMotorSupplyCurrentLimit = Amps.of(40);
			public static final Current shooterMotorCurrentLowerThreshold = Amps.of(30);
			public static final Current shooterMotorStatorCurrentLimit = Amps.of(60);

			// Torque PI
			public static final double openLoopRamp = 0.1;
			public static final double closedLoopRamp = 0.1;

			public static final double shooterMotorVelocityKP = 50;
			public static final double shooterMotorVelocityKI = 0.0;
			public static final double shooterMotorVelocityKD = 0.0;
			public static final double shooterMotorVelocityKS = 0.45181;
			public static final double shooterMotorVelocityKV = 0.7;
			public static final double shooterMotorVelocityKA = 0.025;

			public static TalonFXConfiguration getShooterMotorConfiguration() {

				m_shooterMotorConfig.MotorOutput.Inverted = shooterMotorInverted;
				m_shooterMotorConfig.MotorOutput.NeutralMode = shooterMotorNeutralMode;

				m_shooterMotorConfig.Feedback.SensorToMechanismRatio = 1 / shooterGearRatio;

				// Current Limiting
				m_shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
				m_shooterMotorConfig.CurrentLimits.SupplyCurrentLimit =
						shooterMotorSupplyCurrentLimit.in(Amps);
				m_shooterMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
						shooterMotorCurrentThresholdTime.in(Seconds);
				m_shooterMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
						shooterMotorCurrentLowerThreshold.in(Amps);
				m_shooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
				m_shooterMotorConfig.CurrentLimits.StatorCurrentLimit =
						shooterMotorStatorCurrentLimit.in(Amps);

				// PID Config
				m_shooterMotorConfig.Slot0.kP = shooterMotorVelocityKP;
				m_shooterMotorConfig.Slot0.kI = shooterMotorVelocityKI;
				m_shooterMotorConfig.Slot0.kD = shooterMotorVelocityKD;
				m_shooterMotorConfig.Slot0.kS = shooterMotorVelocityKS;
				m_shooterMotorConfig.Slot0.kV = shooterMotorVelocityKV;
				m_shooterMotorConfig.Slot0.kA = shooterMotorVelocityKA;

				// Open and Closed Loop Ramping
				m_shooterMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
				m_shooterMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

				m_shooterMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
				m_shooterMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;

				m_shooterMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;
				m_shooterMotorConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

				return m_shooterMotorConfig;
			}

			public static TalonFXConfiguration getShooterBangBangConfiguration() {
				TalonFXConfiguration config = getShooterMotorConfiguration();

				config.Slot0.kP = 10000;
				config.Slot0.kD = 0.0;
				config.Slot0.kV = 0.0;
				config.Slot0.kA = 0.0;
				config.TorqueCurrent.PeakForwardTorqueCurrent = 60.0;
				config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
				config.MotorOutput.PeakForwardDutyCycle = 1.0;
				config.MotorOutput.PeakReverseDutyCycle = 0.0;

				return config;
			}

		}
	}

	public class TurretConstants {

		public static final int turretMotorCanID = 43;
		public static final int encoder1Port = 1;
		public static final int encoder2Port = 2;

		public static final int ringGearTeeth = 102;
		public static final int encoderPinion1Teeth = 18;
		public static final int encoderPinion2Teeth = 19;

		public static final double motorToTurretRatio =
				10.0 / 54.0 * encoderPinion1Teeth / ringGearTeeth;
		// translation from robot center to turret center, climber is forward
		public static final Transform2d turretPositionOffset =
				new Transform2d(Inches.of(2.5), Inches.of(-5.945), Rotation2d.kZero);
		public static final double turretPositionToleranceDegrees = 1.5;
		// stow angle and threshold
		public static final Angle turretStowAngle = Degrees.of(-90);

		// at CW limit
		public static final Angle encoder1ZeroPoint = Degrees.of(48.831);
		public static final Angle encoder2ZeroPoint = Degrees.of(107.516);

		// discrepancy threshold between encoders to accept solution
		public static final Angle goodMatchTolerance = Degrees.of(10);
		public static final Angle acceptableMatchTolerance = Degrees.of(25);
		// minimum difference between two possible solutions to avoid ambiguity
		public static final Angle ambiguityTolerance = Degrees.of(2);

		// abs for both
		public static final Angle turretRange = Degrees.of(270);
		public static final Angle turretSoftRange = Degrees.of(260);

		public static class MotorConfigs {
			public static final Time closedLoopRamp = Seconds.of(0.25);

			public static final Current statorCurrentLimit = Amps.of(60);
			public static final Current supplyCurrentLimit = Amps.of(40);
			// set current limit to 30 amps if supply current limit is exceeded for more than 0.5 seconds
			public static final Current supplyCurrentLowerLimit = Amps.of(30);
			public static final Time supplyCurrentLowerTime = Seconds.of(0.5);

			public static final Current statorForwardCurrentLimit = Amps.of(60);
			public static final Current statorReverseCurrentLimit = Amps.of(60);

			public static final Angle forwardSoftLimit = turretRange.div(2).minus(Degrees.of(5));
			public static final Angle reverseSoftLimit = turretRange.div(-2).plus(Degrees.of(5));

			public static final Frequency timeSyncFreq = Hertz.of(250);

			public static final double turretKA = 0; // voltage per unit of acceleration
			public static final double turretKG = 0; // voltage to overcome gravity
			public static final double turretKS = 8; // voltage to overcome static friction
			public static final double turretKV = 0.1; // voltage per unit of requested velocity
			public static final double turretKP = 80.0;
			public static final double turretKI = 0;
			public static final double turretKD = 3;

			public static final double motionMagicCruiseVelocity = 5;
			public static final double motionMagicCruiseAcceleration = 5;
			public static final double motionMagicJerk = 5;
			public static final double motionMagickV = 0;
			public static final double motionMagickA = 0;

			public static TalonFXConfiguration getTurretMotorConfig() {
				TalonFXConfiguration m_TurretMotorConfig = new TalonFXConfiguration();

				m_TurretMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp.in(Seconds);

				m_TurretMotorConfig.Feedback.SensorToMechanismRatio = 1 / motorToTurretRatio;

				m_TurretMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;

				m_TurretMotorConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit.in(Amps);
				m_TurretMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

				m_TurretMotorConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit.in(Amps);
				m_TurretMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
				m_TurretMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
						supplyCurrentLowerLimit.in(Amps);
				m_TurretMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
						supplyCurrentLowerTime.in(Seconds);

				m_TurretMotorConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);
				m_TurretMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
				m_TurretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

				m_TurretMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
				m_TurretMotorConfig.Slot0.StaticFeedforwardSign =
						StaticFeedforwardSignValue.UseVelocitySign;
				m_TurretMotorConfig.Slot0.kA = turretKA; // tune third
				m_TurretMotorConfig.Slot0.kS = turretKS; // tune second
				m_TurretMotorConfig.Slot0.kV = turretKV; // tune third
				m_TurretMotorConfig.Slot0.kP = turretKP; // tune fourth
				m_TurretMotorConfig.Slot0.kI = turretKI; // tune only if needed
				m_TurretMotorConfig.Slot0.kD = turretKD; // tune fifth

				m_TurretMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
						statorForwardCurrentLimit.in(Amps);
				m_TurretMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
						statorReverseCurrentLimit.in(Amps);

				m_TurretMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
				m_TurretMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
						forwardSoftLimit.in(Rotations);
				m_TurretMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
				m_TurretMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
						reverseSoftLimit.in(Rotations);

				return m_TurretMotorConfig;
			}
		}
	}
}
