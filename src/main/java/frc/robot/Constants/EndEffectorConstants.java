// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class EndEffectorConstants {

	public class Intake {
		public static final int intakeMotorCanID = 52; // Kraken X44
		public static final int pivotCanID = 51; // Kraken X60

		public static final int pivotEncoderPort = 0;

		public static final Angle pivotZeroPoint = Rotations.of(0);
		public static final Angle encoderOffset = Rotations.of(0);

		public static final Angle pivotEncoderOffset = Rotations.of(0.528); // TODO figure out
		public static final double pivotMotorToMechanismRatio = 1.0 / 48.0; // TODO figure out
		public static final double pivotToleranceAbsolute = 0.05; // rotations, TODO figure out

		// TODO figure out
		public static final Angle pivotDownAngle = Rotations.of(0.5);
		public static final Angle pivotHalfwayAngle = Rotations.of(11.765);
		public static final Angle pivotUpAngle = Rotations.of(17.65);

		public static final AngularVelocity defaultIntakeSpeed = RotationsPerSecond.of(0);
		public static final AngularVelocity intakeRunningSpeed = RotationsPerSecond.of(30);
		public static final AngularVelocity intakeReversingSpeed = RotationsPerSecond.of(-30);

		public static final class MotorConfigs {

			public static final TalonFXConfiguration m_IntakeMotorConfig = new TalonFXConfiguration();
			public static final TalonFXConfiguration m_PivotConfig = new TalonFXConfiguration();

			public static final InvertedValue intakeMotorInvert = InvertedValue.CounterClockwise_Positive;
			public static final InvertedValue pivotInvert = InvertedValue.Clockwise_Positive;

			public static final NeutralModeValue intakeNeutralMode = NeutralModeValue.Brake;
			public static final NeutralModeValue pivotNeutralMode = NeutralModeValue.Brake;

			public static final Frequency timeSyncFreq = Hertz.of(250);

			// Intake current limits
			public static final Current intakeSupplyCurrentLimit = Amps.of(40);
			public static final Current intakeCurrentLowerThreshold = Amps.of(30);

			public static final Current intakeStatorCurrentLimit = Amps.of(60);

			public static final Time intakeCurrentThresholdTime = Seconds.of(0.50);

			// Pivot current limits
			public static final Current pivotSupplyCurrentLimit = Amps.of(40);
			public static final Current pivotCurrentLowerThreshold = Amps.of(30);

			public static final Current pivotStatorCurrentLimit = Amps.of(60);
			public static final Current pivotStatorCurrentLimitForward = Amps.of(60);
			public static final Current pivotStatorCurrentLimitReverse = Amps.of(-60);

			public static final Angle pivotForwardSoftLimit = Rotations.of(18);
			public static final Angle pivotReverseSoftLimit = Rotations.of(-0.25);

			public static final Time pivotCurrentThresholdTime = Seconds.of(0.50);

			// PID
			public static final double openLoopRamp = 0.1;
			public static final double closedLoopRamp = 0.1;

			public static final double intakeVelocityKP = 3.5;
			public static final double intakeVelocityKI = 0.0;
			public static final double intakeVelocityKD = 0.01;
			public static final double intakeVelocityKS = 20;

			// good enough for now
			public static final double pivotKP = 1;
			public static final double pivotKI = 0.0;
			public static final double pivotKD = 0;
			public static final double pivotKS = 4;
			public static final double pivotKG = 6;

			public static final double pivotMMKa = 0;
			public static final double pivotMMKv = 0;
			public static final double pivotMMAcc = 30;
			public static final double pivotMMVel = 50;
			public static final double pivotMMJerk = 500;

			public static TalonFXConfiguration getIntakeMotorConfiguration() {
				m_IntakeMotorConfig.MotorOutput.Inverted = intakeMotorInvert;
				m_IntakeMotorConfig.MotorOutput.NeutralMode = intakeNeutralMode;


				// Current Limiting
				m_IntakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
				m_IntakeMotorConfig.CurrentLimits.SupplyCurrentLimit = intakeSupplyCurrentLimit.in(Amps);
				m_IntakeMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
						intakeCurrentThresholdTime.in(Seconds);
				m_IntakeMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
						intakeCurrentLowerThreshold.in(Amps);

				m_IntakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
				m_IntakeMotorConfig.CurrentLimits.StatorCurrentLimit = intakeStatorCurrentLimit.in(Amps);

				// PID Config
				m_IntakeMotorConfig.Slot0.kP = intakeVelocityKP;
				m_IntakeMotorConfig.Slot0.kI = intakeVelocityKI;
				m_IntakeMotorConfig.Slot0.kD = intakeVelocityKD;
				m_IntakeMotorConfig.Slot0.kS = intakeVelocityKS;

				// Open and Closed Loop Ramping
				m_IntakeMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
				m_IntakeMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

				m_IntakeMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
				m_IntakeMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;

				m_IntakeMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;
				m_IntakeMotorConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

				return m_IntakeMotorConfig;
			}

			public static TalonFXConfiguration getPivotConfiguration() {
				m_PivotConfig.MotorOutput.Inverted = pivotInvert;
				m_PivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

				// Current Limiting
				m_PivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
				m_PivotConfig.CurrentLimits.SupplyCurrentLimit = pivotSupplyCurrentLimit.in(Amps);
				m_PivotConfig.CurrentLimits.SupplyCurrentLowerTime = pivotCurrentThresholdTime.in(Seconds);
				m_PivotConfig.CurrentLimits.SupplyCurrentLowerLimit = pivotCurrentLowerThreshold.in(Amps);

				m_PivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
				m_PivotConfig.CurrentLimits.StatorCurrentLimit = pivotStatorCurrentLimit.in(Amps);
				m_PivotConfig.TorqueCurrent.PeakForwardTorqueCurrent =
						pivotStatorCurrentLimitForward.in(Amps);
				m_PivotConfig.TorqueCurrent.PeakReverseTorqueCurrent =
						pivotStatorCurrentLimitReverse.in(Amps);

				// PID Config
				m_PivotConfig.Slot0.kP = pivotKP;
				m_PivotConfig.Slot0.kI = pivotKI;
				m_PivotConfig.Slot0.kD = pivotKD;
				m_PivotConfig.Slot0.kS = pivotKS;
				m_PivotConfig.Slot0.kG = pivotKG;
				m_PivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
				m_PivotConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

				m_PivotConfig.MotionMagic.MotionMagicAcceleration = pivotMMAcc;
				m_PivotConfig.MotionMagic.MotionMagicCruiseVelocity = pivotMMVel;
				m_PivotConfig.MotionMagic.MotionMagicJerk = pivotMMJerk;
				m_PivotConfig.MotionMagic.MotionMagicExpo_kA = pivotMMKa;
				m_PivotConfig.MotionMagic.MotionMagicExpo_kV = pivotMMKv;

				m_PivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
				m_PivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
						pivotForwardSoftLimit.in(Rotations);
				m_PivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
				m_PivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
						pivotReverseSoftLimit.in(Rotations);

				m_PivotConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

				return m_PivotConfig;
			}
		}
	}

	public class Indexer {
		public static final int indexerMotorCanID = 0;
		public static final int hopperMotorCanID = 1;

		public static final int beamBreakPort = 0;

		public static final AngularVelocity defaultIndexerSpeed = RotationsPerSecond.of(0);
		public static final AngularVelocity runningIndexerSpeed = RotationsPerSecond.of(3);
		public static final AngularVelocity shootingIndexerSpeed = RotationsPerSecond.of(5);
		public static final AngularVelocity reverseIndexerSpeed = RotationsPerSecond.of(-3);

		public static final AngularVelocity defaultHopperSpeed = RotationsPerSecond.of(0);
		public static final AngularVelocity runningHopperSpeed = RotationsPerSecond.of(3);
		public static final AngularVelocity reverseHopperSpeed = RotationsPerSecond.of(-3);

		public class MotorConfigs {
			public static final TalonFXConfiguration m_IndexerMotorConfig = new TalonFXConfiguration();

			public static final InvertedValue indexerMotorInvert =
					InvertedValue.CounterClockwise_Positive;

			public static final NeutralModeValue indexerNeutralMode = NeutralModeValue.Brake;

			public static final Frequency timeSyncFreq = Hertz.of(250);

			// Indexer current limits
			public static final Current indexerSupplyCurrentLimit = Amps.of(40);
			public static final Current indexerCurrentLowerThreshold = Amps.of(30);

			public static final Current indexerStatorCurrentLimit = Amps.of(60);

			public static final Time indexerCurrentThresholdTime = Seconds.of(0.50);

			public static final double openLoopRamp = 0.1;
			public static final double closedLoopRamp = 0.1;

			public static final double indexerVelocityKP = 0.3;
			public static final double indexerVelocityKI = 0.0;
			public static final double indexerVelocityKD = 0.0;
			public static final double indexerVelocityKS = 4.9;

			public static final double indexerPositionKP = 0.75;
			public static final double indexerPositionKI = 0.0;
			public static final double indexerPositionKD = 0.0;

			public static TalonFXConfiguration getIndexerMotorConfig() {
				m_IndexerMotorConfig.MotorOutput.Inverted = indexerMotorInvert;
				m_IndexerMotorConfig.MotorOutput.NeutralMode = indexerNeutralMode;

				m_IndexerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

				// Current Limiting
				m_IndexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
				m_IndexerMotorConfig.CurrentLimits.SupplyCurrentLimit = indexerSupplyCurrentLimit.in(Amps);
				m_IndexerMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
						indexerCurrentThresholdTime.in(Seconds);
				m_IndexerMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
						indexerCurrentLowerThreshold.in(Amps);

				m_IndexerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
				m_IndexerMotorConfig.CurrentLimits.StatorCurrentLimit = indexerStatorCurrentLimit.in(Amps);

				// PID Config
				m_IndexerMotorConfig.Slot0.kP = indexerVelocityKP;
				m_IndexerMotorConfig.Slot0.kI = indexerVelocityKI;
				m_IndexerMotorConfig.Slot0.kD = indexerVelocityKD;
				m_IndexerMotorConfig.Slot0.kS = indexerVelocityKS;

				m_IndexerMotorConfig.Slot1.kP = indexerPositionKP;
				m_IndexerMotorConfig.Slot1.kI = indexerPositionKI;
				m_IndexerMotorConfig.Slot1.kD = indexerPositionKD;
				// Open and Closed Loop Ramping
				m_IndexerMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
				m_IndexerMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

				m_IndexerMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
				m_IndexerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;

				m_IndexerMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;
				m_IndexerMotorConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

				return m_IndexerMotorConfig;
			}
		}
	}
}
