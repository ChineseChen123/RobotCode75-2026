// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class EndEffectorConstants {

    public static final int intakeMotorCanID = 0; // Kraken X44
	public static final int pivotCanID = 0; // Kraken X60
	public static final int indexerMotorCanID = 0;
    
	public static final int intakeEncoderPort = 0;
	public static final int pivotEncoderPort = 0;
	public static final int beamBreakPort = 0;
	public class Intake {
		public static final int intakeMotorCanID = 0; // Kraken X44
		public static final int pivotCanID = 0; // Kraken X60

		public static final int intakeEncoderPort = 0;
		public static final int pivotEncoderPort = 0;

		public static final Angle pivotZeroPoint = Rotations.of(0);

		public static final Angle pivotEncoderOffset = Rotations.of(0); // TODO figure out
		public static final double pivotGearRatio = 1; // TODO figure out
		public static final double pivotToleranceAbsolute = 0.05; // rotations, TODO figure out

		// TODO figure out
		public static final Angle pivotDownAngle = Rotations.of(0);
		public static final Angle pivotUpAngle = Rotations.of(2);

		public static final AngularVelocity intakeSpeed = RotationsPerSecond.of(2);
		public static final AngularVelocity reverseSpeed = RotationsPerSecond.of(-2);

		public static final class MotorConfigs {

		public static final TalonFXConfiguration m_IntakeMotorConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration m_PivotConfig = new TalonFXConfiguration();
		public static final TalonFXConfiguration m_IndexerMotorConfig = new TalonFXConfiguration();
		
        
		public static final InvertedValue intakeMotorInvert = InvertedValue.CounterClockwise_Positive;
		public static final InvertedValue indexMotorInvert = InvertedValue.CounterClockwise_Positive;
		public static final InvertedValue pivotInvert = InvertedValue.CounterClockwise_Positive;
        
		public static final NeutralModeValue intakeNeutralMode = NeutralModeValue.Brake;
		public static final NeutralModeValue indexNeutralMode = NeutralModeValue.Brake;
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

			public static final Angle pivotForwardSoftLimit = Rotations.of(12.5);
			public static final Angle pivotReverseSoftLimit = Rotations.of(-3);

			public static final Time pivotCurrentThresholdTime = Seconds.of(0.50);

			// PID
			public static final double openLoopRamp = 0.1;
			public static final double closedLoopRamp = 0.1;

			public static final double intakeVelocityKP = 0.3;
			public static final double intakeVelocityKI = 0.0;
			public static final double intakeVelocityKD = 0.0;
			public static final double intakeVelocityKS = 4.9;

			public static final double intakePositionKP = 0.75;
			public static final double intakePositionKI = 0.0;
			public static final double intakePositionKD = 0.0;

			public static final double pivotKP = 10;
			public static final double pivotKI = 0.0;
			public static final double pivotKD = 3;
			public static final double pivotKS = 4;
			public static final double pivotKG = 6;

			public static final double pivotMMKa = 0.1;
			public static final double pivotMMKv = 0.15;
			public static final double pivotMMAcc = 30;
			public static final double pivotMMVel = 50;
			public static final double pivotMMJerk = 500;

		public static TalonFXConfiguration getIntakeMotorConfiguration() {
			m_IntakeMotorConfig.MotorOutput.Inverted = intakeMotorInvert;
			m_IntakeMotorConfig.MotorOutput.NeutralMode = intakeNeutralMode;

			m_IntakeMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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

				m_IntakeMotorConfig.Slot1.kP = intakePositionKP;
				m_IntakeMotorConfig.Slot1.kI = intakePositionKI;
				m_IntakeMotorConfig.Slot1.kD = intakePositionKD;

				// Open and Closed Loop Ramping
				m_IntakeMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
				m_IntakeMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

				m_IntakeMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
				m_IntakeMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;

				m_IntakeMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;
				m_IntakeMotorConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

			return m_IntakeMotorConfig;
		}
		public static TalonFXConfiguration getIndexerMotorConfig() {
			m_IndexerMotorConfig.MotorOutput.Inverted = indexMotorInvert;
			m_IndexerMotorConfig.MotorOutput.NeutralMode = indexNeutralMode;

			m_IndexerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

			// Current Limiting
			m_IndexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
			m_IndexerMotorConfig.CurrentLimits.SupplyCurrentLimit = intakeSupplyCurrentLimit.in(Amps);
			m_IndexerMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
					intakeCurrentThresholdTime.in(Seconds);
			m_IndexerMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
					intakeCurrentLowerThreshold.in(Amps);

			m_IndexerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
			m_IndexerMotorConfig.CurrentLimits.StatorCurrentLimit = intakeStatorCurrentLimit.in(Amps);

			// PID Config
			m_IndexerMotorConfig.Slot0.kP = intakeVelocityKP;
			m_IndexerMotorConfig.Slot0.kI = intakeVelocityKI;
			m_IndexerMotorConfig.Slot0.kD = intakeVelocityKD;
			m_IndexerMotorConfig.Slot0.kS = intakeVelocityKS;

			m_IndexerMotorConfig.Slot1.kP = intakePositionKP;
			m_IndexerMotorConfig.Slot1.kI = intakePositionKI;
			m_IndexerMotorConfig.Slot1.kD = intakePositionKD;

			// Open and Closed Loop Ramping
			m_IndexerMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
			m_IndexerMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

			m_IndexerMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
			m_IndexerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;

			m_IndexerMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;
			m_IndexerMotorConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

			return m_IndexerMotorConfig;
		}
        
        public static TalonFXConfiguration getPivotConfiguration() {
			m_PivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
}
