// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
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
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class ShooterConstants {

	public class Shooter {
		// Kraken X60s
		public static final int shooterMotor1CanID = 0;
		public static final int shooterMotor2CanID = 0;

		public static final double shooterGearRatio = 1;

		public static final Distance shooterWheelDiameter = Inches.of(4);

		public static final AngularVelocity defaultShooterSpeed = RotationsPerSecond.of(0);
		public static final AngularVelocity reverseShooterSpeed = RotationsPerSecond.of(-5);

		public static final class MotorConfigs {

			public static final TalonFXConfiguration m_shooterMotor1Config = new TalonFXConfiguration();
			public static final TalonFXConfiguration m_shooterMotor2Config = new TalonFXConfiguration();

			// Neutral modes and inverts
			public static final InvertedValue shooterMotor1Inverted =
					InvertedValue.CounterClockwise_Positive;
			public static final InvertedValue shooterMotor2Inverted = InvertedValue.Clockwise_Positive;

			public static final NeutralModeValue shooterMotor1NeutralMode = NeutralModeValue.Coast;
			public static final NeutralModeValue shooterMotor2NeutralMode = NeutralModeValue.Coast;

			public static final Frequency timeSyncFreq = Hertz.of(250);

			public static final Time shooterMotor1CurrentThresholdTime = Seconds.of(0.5);
			public static final Time shooterMotor2CurrentThresholdTime = Seconds.of(0.5);

			public static final Current shooterMotor1SupplyCurrentLimit = Amps.of(40);
			public static final Current shooterMotor1CurrentLowerThreshold = Amps.of(30);
			public static final Current shooterMotor1StatorCurrentLimit = Amps.of(60);

			public static final Current shooterMotor2SupplyCurrentLimit = Amps.of(40);
			public static final Current shooterMotor2CurrentLowerThreshold = Amps.of(30);
			public static final Current shooterMotor2StatorCurrentLimit = Amps.of(60);

			// Torque PI
			public static final double openLoopRamp = 0.1;
			public static final double closedLoopRamp = 0.1;

			public static final double shooterMotor1VelocityKP = 0.3;
			public static final double shooterMotor1VelocityKI = 0.0;
			public static final double shooterMotor1VelocityKD = 0.0;
			public static final double shooterMotor1VelocityKS = 4.9;

			public static final double shooterMotor2VelocityKP = 0.3;
			public static final double shooterMotor2VelocityKI = 0.0;
			public static final double shooterMotor2VelocityKD = 0.0;
			public static final double shooterMotor2VelocityKS = 4.9;

			public static TalonFXConfiguration getShooterMotor1MotorConfiguration() {

				m_shooterMotor1Config.MotorOutput.Inverted = shooterMotor1Inverted;
				m_shooterMotor1Config.MotorOutput.NeutralMode = shooterMotor1NeutralMode;

				// Current Limiting
				m_shooterMotor1Config.CurrentLimits.SupplyCurrentLimitEnable = true;
				m_shooterMotor1Config.CurrentLimits.SupplyCurrentLimit =
						shooterMotor1SupplyCurrentLimit.in(Amps);
				m_shooterMotor1Config.CurrentLimits.SupplyCurrentLowerTime =
						shooterMotor1CurrentThresholdTime.in(Seconds);
				m_shooterMotor1Config.CurrentLimits.SupplyCurrentLowerLimit =
						shooterMotor1CurrentLowerThreshold.in(Amps);

				m_shooterMotor1Config.CurrentLimits.StatorCurrentLimitEnable = true;
				m_shooterMotor1Config.CurrentLimits.StatorCurrentLimit =
						shooterMotor1StatorCurrentLimit.in(Amps);

				// PID Config
				m_shooterMotor1Config.Slot0.kP = shooterMotor1VelocityKP;
				m_shooterMotor1Config.Slot0.kI = shooterMotor1VelocityKI;
				m_shooterMotor1Config.Slot0.kD = shooterMotor1VelocityKD;
				m_shooterMotor1Config.Slot0.kS = shooterMotor1VelocityKS;

				// Open and Closed Loop Ramping
				m_shooterMotor1Config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
				m_shooterMotor1Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

				m_shooterMotor1Config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
				m_shooterMotor1Config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;

				m_shooterMotor1Config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;
				m_shooterMotor1Config.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

				return m_shooterMotor1Config;
			}

			public static TalonFXConfiguration getShooterMotor2MotorConfiguration() {

				m_shooterMotor2Config.MotorOutput.Inverted = shooterMotor2Inverted;
				m_shooterMotor2Config.MotorOutput.NeutralMode = shooterMotor2NeutralMode;

				// Current Limiting
				m_shooterMotor2Config.CurrentLimits.SupplyCurrentLimitEnable = true;
				m_shooterMotor2Config.CurrentLimits.SupplyCurrentLimit =
						shooterMotor2SupplyCurrentLimit.in(Amps);
				m_shooterMotor2Config.CurrentLimits.SupplyCurrentLowerTime =
						shooterMotor2CurrentThresholdTime.in(Seconds);
				m_shooterMotor2Config.CurrentLimits.SupplyCurrentLowerLimit =
						shooterMotor2CurrentLowerThreshold.in(Amps);
				m_shooterMotor2Config.CurrentLimits.StatorCurrentLimitEnable = true;
				m_shooterMotor2Config.CurrentLimits.StatorCurrentLimit =
						shooterMotor2StatorCurrentLimit.in(Amps);

				// PID Config
				m_shooterMotor2Config.Slot0.kP = shooterMotor2VelocityKP;
				m_shooterMotor2Config.Slot0.kI = shooterMotor2VelocityKI;
				m_shooterMotor2Config.Slot0.kD = shooterMotor2VelocityKD;
				m_shooterMotor2Config.Slot0.kS = shooterMotor2VelocityKS;

				// Open and Closed Loop Ramping
				m_shooterMotor2Config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
				m_shooterMotor2Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

				m_shooterMotor2Config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
				m_shooterMotor2Config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;

				m_shooterMotor2Config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;
				m_shooterMotor2Config.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

				return m_shooterMotor2Config;
			}
		}
	}

	public class Turret {

		public static final int turretMotorCanID = 0;
		public static final int encoder1Port = 0;
		public static final int encoder2Port = 0;

		public static final int ringGearTeeth = 102;
		public static final int encoderPinion1Teeth = 18;
		public static final int encoderPinion2Teeth = 19;

		public static final double motorToMechanismRatio = 1.0 / 1.0;
		// translation from robot center to turret center, climber is forward
		public static final Translation2d turretPositionOffset =
				new Translation2d(Inches.of(0), Inches.of(0));

		// at CW limit
		public static final Angle encoder1Offset = Rotations.of(0);
		public static final Angle encoder2Offset = Rotations.of(0);
		// discrepancy threshold between encoders to accept solution
		public static final Angle matchTolerance = Rotations.of(0.005);
		// minimum difference between two possible solutions to avoid ambiguity
		public static final Angle ambiguityTolerance = Rotations.of(0.001);

		// abs for both
		public static final Angle turretRingGearRange = Degrees.of(270);
		public static final Angle turretMotorRange = Rotations.of(100);

		public static class MotorConfigs {
			public static final Time closedLoopRamp = Seconds.of(0.25);

			public static final Current statorCurrentLimit = Amps.of(60);
			public static final Current supplyCurrentLimit = Amps.of(40);
			// set current limit to 30 amps if supply current limit is exceeded for more than 0.5 seconds
			public static final Current supplyCurrentLowerLimit = Amps.of(30);
			public static final Time supplyCurrentLowerTime = Seconds.of(0.5);

			public static final Current statorForwardCurrentLimit = Amps.of(100);
			public static final Current statorReverseCurrentLimit = Amps.of(100);

			public static final Angle forwardSoftLimit = Rotations.of(50); // TODO test
			public static final Angle reverseSoftLimit = Rotations.of(-50);

			public static final Frequency timeSyncFreq = Hertz.of(250);

			public static final double kA = 0.1; // current per unit of acceleration
			public static final double kG = 0; // current to overcome gravity
			public static final double kS = 0; // current to overcome static friction
			public static final double kV = 0.1; // current per unit of requested velocity
			public static final double kP = 5;
			public static final double kI = 0;
			public static final double kD = 2;

			public static final double motionMagicCruiseVelocity = 40;
			public static final double motionMagicCruiseAcceleration = 10;
			public static final double motionMagickV = 0.12;
			public static final double motionMagickA = 0.1;

			public static TalonFXConfiguration getTurretMotorConfig() {
				TalonFXConfiguration m_TurretMotorConfig = new TalonFXConfiguration();

				m_TurretMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp.in(Seconds);

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

				m_TurretMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
				m_TurretMotorConfig.Slot0.StaticFeedforwardSign =
						StaticFeedforwardSignValue.UseVelocitySign;
				m_TurretMotorConfig.Slot0.kA = kA; // tune third
				m_TurretMotorConfig.Slot0.kG = kG; // tune first
				m_TurretMotorConfig.Slot0.kS = kS; // tune second
				m_TurretMotorConfig.Slot0.kV = kV; // tune third
				m_TurretMotorConfig.Slot0.kP = kP; // tune fourth
				m_TurretMotorConfig.Slot0.kI = kI; // tune only if needed
				m_TurretMotorConfig.Slot0.kD = kD; // tune fifth

				m_TurretMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
						statorForwardCurrentLimit.in(Amps);
				m_TurretMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
						statorReverseCurrentLimit.in(Amps);

				m_TurretMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
				m_TurretMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
						forwardSoftLimit.in(Rotations);
				m_TurretMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
				m_TurretMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
						reverseSoftLimit.in(Rotations);

				return m_TurretMotorConfig;
			}
		}
	}
}
