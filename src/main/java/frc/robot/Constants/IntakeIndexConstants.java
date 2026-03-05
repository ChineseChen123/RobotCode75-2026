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
public class IntakeIndexConstants {

	public class IntakeConstants {
		public static final int intakeMotorCanID = 52; // Kraken X44
		public static final int pivotCanID = 51; // Kraken X60

		public static final int pivotEncoderPort = 0;

		public static final Angle pivotZeroPoint = Rotations.of(0);

		public static final Angle pivotEncoderOffset = Rotations.of(0.528);
		public static final double pivotMotorToMechanismRatio = 1.0 / 48.0;
		public static final double pivotToleranceAbsolute = 0.05; // rotations // TODO figure out

		// TODO figure out
		public static final Angle pivotDownAngle = Rotations.of(-0.04);
		public static final Angle pivotHalfwayAngle = Rotations.of(0.20);
		public static final Angle pivotUpAngle = Rotations.of(0.4);

		public static final AngularVelocity defaultIntakeSpeed = RotationsPerSecond.of(0);
		public static final AngularVelocity intakeRunningSpeed = RotationsPerSecond.of(40);
		public static final AngularVelocity intakeReversingSpeed = RotationsPerSecond.of(-30);

		public static final class MotorConfigs {

			public static final TalonFXConfiguration m_IntakeMotorConfig = new TalonFXConfiguration();
			public static final TalonFXConfiguration m_PivotConfig = new TalonFXConfiguration();

			public static final InvertedValue intakeMotorInvert = InvertedValue.Clockwise_Positive;
			public static final InvertedValue pivotInvert = InvertedValue.Clockwise_Positive;

			public static final NeutralModeValue intakeNeutralMode = NeutralModeValue.Brake;
			public static final NeutralModeValue pivotNeutralMode = NeutralModeValue.Brake;

			public static final Frequency timeSyncFreq = Hertz.of(250);

			// Intake current limits
			public static final Current intakeSupplyCurrentLimit = Amps.of(40);
			public static final Current intakeCurrentLowerThreshold = Amps.of(30);

			public static final Current intakeStatorCurrentLimit = Amps.of(80);

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

			// good enough for now
			public static final double intakeVelocityKP = 3.5;
			public static final double intakeVelocityKI = 0.0;
			public static final double intakeVelocityKD = 0.01;
			public static final double intakeVelocityKS = 20;

			// good enough for now
			public static final double pivotKP = 1000;
			public static final double pivotKI = 0.0;
			public static final double pivotKD = 7;
			public static final double pivotKS = 20;
			public static final double pivotKG = 9;

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

				m_IntakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
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

				m_PivotConfig.Feedback.SensorToMechanismRatio = 1 / pivotMotorToMechanismRatio;

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

			public static TalonFXConfiguration getIntakeBangBangConfiguration() {
				TalonFXConfiguration config = getIntakeMotorConfiguration();

				config.Slot0.kP = 10000;
				config.Slot0.kD = 0.0;
				config.Slot0.kV = 0.0;
				config.Slot0.kA = 0.0;
				config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
				config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
				config.MotorOutput.PeakForwardDutyCycle = 1.0;
				config.MotorOutput.PeakReverseDutyCycle = 0.0;

				return config;
			}
		}
	}

	public class IndexerConstants {

		public static final int indexerMotorCanID = 54;
		public static final int hopperMotorCanID = 53;

		public static final int beamBreakPort = 3;

		public static final AngularVelocity defaultIndexerSpeed = RotationsPerSecond.of(0);
		public static final AngularVelocity runningIndexerSpeed = RotationsPerSecond.of(15);
		public static final AngularVelocity shootingIndexerSpeed = RotationsPerSecond.of(60);
		public static final AngularVelocity reverseIndexerSpeed = RotationsPerSecond.of(-15);

		public static final AngularVelocity defaultHopperSpeed = RotationsPerSecond.of(0);
		public static final AngularVelocity runningHopperSpeed = RotationsPerSecond.of(20);
		public static final AngularVelocity reverseHopperSpeed = RotationsPerSecond.of(-15);

		public class MotorConfigs {
			public static final TalonFXConfiguration m_IndexerMotorConfig = new TalonFXConfiguration();
			public static final TalonFXConfiguration m_HopperMotorConfig = new TalonFXConfiguration();

			public static final InvertedValue indexerMotorInvert = InvertedValue.Clockwise_Positive;
			public static final InvertedValue hopperMotorInvert = InvertedValue.CounterClockwise_Positive;

			public static final NeutralModeValue indexerNeutralMode = NeutralModeValue.Brake;
			public static final NeutralModeValue hopperNeutralMode = NeutralModeValue.Brake;

			public static final Frequency timeSyncFreq = Hertz.of(250);

			// Indexer current limits
			public static final Current indexerSupplyCurrentLimit = Amps.of(40);
			public static final Current indexerCurrentLowerThreshold = Amps.of(30);
			public static final Current hopperSupplyCurrentLimit = Amps.of(40);
			public static final Current hopperCurrentLowerThreshold = Amps.of(30);

			public static final Current indexerStatorCurrentLimit = Amps.of(80);
			public static final Current hopperStatorCurrentLimit = Amps.of(60);

			public static final Time indexerCurrentThresholdTime = Seconds.of(0.50);
			public static final Time hopperCurrentThresholdTime = Seconds.of(0.50);

			public static final double openLoopRamp = 0.1;
			public static final double closedLoopRamp = 0.1;

			public static final double indexerVelocityKP = 1000;
			public static final double indexerVelocityKI = 0.0;
			public static final double indexerVelocityKD = 0.0;
			public static final double indexerVelocityKS = 25;
			public static final double indexerVelocityKV = 1.25;

			public static final double hopperVelocityKP = 10;
			public static final double hopperVelocityKI = 0.0;
			public static final double hopperVelocityKD = 0.0;
			public static final double hopperVelocityKS = 1.5;

			public static TalonFXConfiguration getIndexerMotorConfig() {
				m_IndexerMotorConfig.MotorOutput.Inverted = indexerMotorInvert;
				m_IndexerMotorConfig.MotorOutput.NeutralMode = indexerNeutralMode;

				// Current Limiting
				m_IndexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
				m_IndexerMotorConfig.CurrentLimits.SupplyCurrentLimit = indexerSupplyCurrentLimit.in(Amps);
				m_IndexerMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
						indexerCurrentThresholdTime.in(Seconds);
				m_IndexerMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
						indexerCurrentLowerThreshold.in(Amps);

				m_IndexerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
				m_IndexerMotorConfig.CurrentLimits.StatorCurrentLimit = indexerStatorCurrentLimit.in(Amps);

				// PID Config
				m_IndexerMotorConfig.Slot0.kP = indexerVelocityKP;
				m_IndexerMotorConfig.Slot0.kI = indexerVelocityKI;
				m_IndexerMotorConfig.Slot0.kD = indexerVelocityKD;
				m_IndexerMotorConfig.Slot0.kS = indexerVelocityKS;
				m_IndexerMotorConfig.Slot0.kV = indexerVelocityKV;

				// Open and Closed Loop Ramping
				m_IndexerMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
				m_IndexerMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

				m_IndexerMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
				m_IndexerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;

				m_IndexerMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;
				m_IndexerMotorConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

				return m_IndexerMotorConfig;
			}

			public static TalonFXConfiguration getHopperMotorConfig() {
				m_HopperMotorConfig.MotorOutput.Inverted = hopperMotorInvert;
				m_HopperMotorConfig.MotorOutput.NeutralMode = hopperNeutralMode;

				// Current Limiting
				m_HopperMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
				m_HopperMotorConfig.CurrentLimits.SupplyCurrentLimit = hopperSupplyCurrentLimit.in(Amps);
				m_HopperMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
						hopperCurrentThresholdTime.in(Seconds);
				m_HopperMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
						hopperCurrentLowerThreshold.in(Amps);

				m_HopperMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
				m_HopperMotorConfig.CurrentLimits.StatorCurrentLimit = hopperStatorCurrentLimit.in(Amps);

				// PID Config
				m_HopperMotorConfig.Slot0.kP = hopperVelocityKP;
				m_HopperMotorConfig.Slot0.kI = hopperVelocityKI;
				m_HopperMotorConfig.Slot0.kD = hopperVelocityKD;
				m_HopperMotorConfig.Slot0.kS = hopperVelocityKS;

				// Open and Closed Loop Ramping
				m_HopperMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
				m_HopperMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

				m_HopperMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
				m_HopperMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;

				m_HopperMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;
				m_HopperMotorConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

				return m_HopperMotorConfig;
			}

			public static TalonFXConfiguration getIndexerBangBangConfiguration() {
				TalonFXConfiguration config = getIndexerMotorConfig();

				config.Slot0.kP = 10000;
				config.Slot0.kD = 0.0;
				config.Slot0.kV = 0.0;
				config.Slot0.kA = 0.0;
				config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
				config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
				config.MotorOutput.PeakForwardDutyCycle = 1.0;
				config.MotorOutput.PeakReverseDutyCycle = 0.0;

				return config;
			}
		}
	}
}
