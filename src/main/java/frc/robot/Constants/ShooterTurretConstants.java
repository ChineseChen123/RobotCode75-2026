// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class ShooterTurretConstants {

	public static final boolean useVirtualTarget = false;
	public static final int virtualTargetSolveIterations = 6;

	public class ShooterConstants {
		// Kraken X60s
		public static final int shooterMotor1CanID = 41;
		public static final int shooterMotor2CanID = 42;

		public static final double shooterGearRatio = 1.0; // 1.5 / 1.0

		public static final Distance shooterWheelDiameter = Inches.of(4);

		public static final AngularVelocity defaultShooterSpeed = RotationsPerSecond.of(0);
		public static final AngularVelocity reverseShooterSpeed = RotationsPerSecond.of(-10);

		public static final double shooterVelocityTolerance = 50.0 / 60.0; // rotations per second

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
		public static final Angle shooterAngleWithHorizontal = Degrees.of(12);
		public static final Distance shooterHeight = Inches.of(19.5);

		public static final class MotorConfigs {

			public static final TalonFXConfiguration m_shooterMotor1Config = new TalonFXConfiguration();
			public static final TalonFXConfiguration m_shooterMotor2Config = new TalonFXConfiguration();

			// Neutral modes and inverts
			public static final InvertedValue shooterMotorInverted = InvertedValue.Clockwise_Positive;

			public static final NeutralModeValue shooterMotorNeutralMode = NeutralModeValue.Coast;

			public static final Frequency timeSyncFreq = Hertz.of(250);

			public static final Time shooterMotorCurrentThresholdTime = Seconds.of(0.5);

			public static final Current shooterMotorSupplyCurrentLimit = Amps.of(40);
			public static final Current shooterMotorCurrentLowerThreshold = Amps.of(30);
			public static final Current shooterMotorStatorCurrentLimit = Amps.of(80);

			// Torque PI
			public static final double openLoopRamp = 0.1;
			public static final double closedLoopRamp = 0.1;

			public static final double shooterMotorVelocityKP = 25;
			public static final double shooterMotorVelocityKI = 0.0;
			public static final double shooterMotorVelocityKD = 0.0;
			public static final double shooterMotorVelocityKS = 0.3;

			public static TalonFXConfiguration getShooterMotorConfiguration() {

				m_shooterMotor1Config.MotorOutput.Inverted = shooterMotorInverted;
				m_shooterMotor1Config.MotorOutput.NeutralMode = shooterMotorNeutralMode;

				m_shooterMotor1Config.Feedback.SensorToMechanismRatio = 1 / shooterGearRatio;

				// Current Limiting
				m_shooterMotor1Config.CurrentLimits.SupplyCurrentLimitEnable = true;
				m_shooterMotor1Config.CurrentLimits.SupplyCurrentLimit =
						shooterMotorSupplyCurrentLimit.in(Amps);
				m_shooterMotor1Config.CurrentLimits.SupplyCurrentLowerTime =
						shooterMotorCurrentThresholdTime.in(Seconds);
				m_shooterMotor1Config.CurrentLimits.SupplyCurrentLowerLimit =
						shooterMotorCurrentLowerThreshold.in(Amps);
				m_shooterMotor1Config.CurrentLimits.StatorCurrentLimitEnable = true;
				m_shooterMotor1Config.CurrentLimits.StatorCurrentLimit =
						shooterMotorStatorCurrentLimit.in(Amps);

				// PID Config
				m_shooterMotor1Config.Slot0.kP = shooterMotorVelocityKP;
				m_shooterMotor1Config.Slot0.kI = shooterMotorVelocityKI;
				m_shooterMotor1Config.Slot0.kD = shooterMotorVelocityKD;
				m_shooterMotor1Config.Slot0.kS = shooterMotorVelocityKS;

				// Open and Closed Loop Ramping
				m_shooterMotor1Config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
				m_shooterMotor1Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

				m_shooterMotor1Config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
				m_shooterMotor1Config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;

				m_shooterMotor1Config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;
				m_shooterMotor1Config.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

				return m_shooterMotor1Config;
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
		public static final Translation2d turretPositionOffset =
				new Translation2d(Inches.of(2.5), Inches.of(-5.945));
		public static final double turretPositionToleranceDegrees = 1.5;
		// stow angle and threshold
		public static final Angle turretStowAngle = Degrees.of(-90);

		// at CW limit
		public static final Angle encoder1ZeroPoint = Degrees.of(91.6);
		public static final Angle encoder2ZeroPoint = Degrees.of(62.8);

		// discrepancy threshold between encoders to accept solution
		public static final Angle matchTolerance = Degrees.of(8);
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

			public static final double kA = 0; // voltage per unit of acceleration
			public static final double kG = 0; // voltage to overcome gravity
			public static final double kS = 3.2; // voltage to overcome static friction
			public static final double kV = 0; // voltage per unit of requested velocity
			public static final double kP = 70;
			public static final double kI = 0;
			public static final double kD = 3;

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
				m_TurretMotorConfig.Slot0.kA = kA; // tune third
				m_TurretMotorConfig.Slot0.kS = kS; // tune second
				m_TurretMotorConfig.Slot0.kV = kV; // tune third
				m_TurretMotorConfig.Slot0.kP = kP; // tune fourth
				m_TurretMotorConfig.Slot0.kI = kI; // tune only if needed
				m_TurretMotorConfig.Slot0.kD = kD; // tune fifth

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
