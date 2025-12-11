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

	// CAN IDs
	public static final int coralMotorCanID = 43; // TalonFXS into minion
	public static final int pivotCanID = 44; // Kraken X60
	public static final int algaeMotorCanID = 45; // Kraken X44

	public static final int coralBeamBreakPort = 2; // Banner BeamBreak
	public static final int algaePivotEncoderPort = 4; // REV Through Bore
	public static final int algaeLidarSensorPort = 5; // Pololu LIDAR sensor

	public static final Angle algaePivotZeroPoint = Rotations.of(0);
	public static final Angle algaeEncoderOffset = Rotations.of(0);

	public static final double coralMotorGearRatio = 4.0;
	public static final double algaeMotorGearRatio = 25.0;
	public static final double pivotMotorGearRatio = (25.0 * 50.0) / 26.0; // ~48.076

	// Coral Intake speeds
	public static final AngularVelocity coralScoreSpeed = RotationsPerSecond.of(50);
	public static final AngularVelocity coralIntakeSpeed = RotationsPerSecond.of(30);
	public static final AngularVelocity coralScoreSpeedL1 = RotationsPerSecond.of(7);
	public static final AngularVelocity coralReverseSpeed = RotationsPerSecond.of(-30);
	public static final Angle coralRotationsAfterIntake = Rotations.of(1.25);
	public static final double coralScoreDelay = 0.15;

	// Algae Pivot positions
	public static final Angle pivotHomePosition = Rotations.of(11);
	public static final Angle pivotGroundIntakePosition = Rotations.of(-3.5); // -3.4709401709
	public static final Angle pivotDeAlgifyPosition = Rotations.of(6.7);
	public static final Angle pivotProcessorPosition = Rotations.of(1);
	public static final Angle pivotNetPosition = Rotations.of(9.75);
	public static final Angle pivotEncoderOffset = Rotations.of(0.174);
	public static final double pivotDeAlgaefyDelay = 0.4;

	// Algae Intake speeds
	public static final AngularVelocity algaeIntakeSpeed = RotationsPerSecond.of(125.5);
	public static final AngularVelocity algaeScoreSpeed = RotationsPerSecond.of(-125.5);
	public static final AngularVelocity algaeHoldSpeed = RotationsPerSecond.of(125.5);
	public static final Current algaeHoldCurrent = Amps.of(25);

	// Position tolerances
	public static final double coralPositionTolerance = 0.2;
	public static final double algaePivotTolerance = 0.9;

	public static final class MotorConfigs {

		public static final TalonFXSConfiguration m_CoralMotorConfig = new TalonFXSConfiguration();
		public static final TalonFXConfiguration m_AlgaeMotorConfig = new TalonFXConfiguration();
		public static final TalonFXConfiguration m_PivotConfig = new TalonFXConfiguration();

		// Neutral modes and inverts
		public static final InvertedValue coralMotorInvert = InvertedValue.CounterClockwise_Positive;
		public static final InvertedValue algaeMotorInvert = InvertedValue.CounterClockwise_Positive;
		public static final InvertedValue pivotInvert = InvertedValue.CounterClockwise_Positive;

		public static final NeutralModeValue coralMotorNuetralMode = NeutralModeValue.Brake;
		public static final NeutralModeValue algaeMotorNuetralMode = NeutralModeValue.Brake;
		public static final NeutralModeValue pivotNeutralMode = NeutralModeValue.Brake;

		public static final Frequency timeSyncFreq = Hertz.of(250);

		// Coral Intake current limiting
		public static final Current coralSupplyCurrentLimit = Amps.of(40);
		public static final Current coralCurrentLowerThreshold = Amps.of(30);

		public static final Current coralStatorCurrentLimit = Amps.of(60);

		public static final Time coralCurrentThresholdTime = Seconds.of(0.50);

		// Algae Intake current limiting
		public static final Current algaeSupplyCurrentLimit = Amps.of(50);
		public static final Current algaeLowerCurrentThreshold = Amps.of(30);

		public static final Current algaeStatorCurrentLimit = Amps.of(50);
		public static final Current algaeStatorCurrentLimitForward = Amps.of(50);
		public static final Current algaeStatorCurrentLimitReverse = Amps.of(-50);

		public static final Time algaeCurrentThresholdTime = Seconds.of(1);

		// Algae Pivot current limits
		public static final Current pivotSupplyCurrentLimit = Amps.of(40);
		public static final Current pivotCurrentLowerThreshold = Amps.of(30);

		public static final Current pivotStatorCurrentLimit = Amps.of(60);
		public static final Current pivotStatorCurrentLimitForward = Amps.of(60);
		public static final Current pivotStatorCurrentLimitReverse = Amps.of(-60);

		public static final Angle pivotForwardSoftLimit = Rotations.of(12.5);
		public static final Angle pivotReverseSoftLimit = Rotations.of(-3);

		public static final Time pivotCurrentThresholdTime = Seconds.of(0.50);

		// Torque PID
		public static final double openLoopRamp = 0.1;
		public static final double closedLoopRamp = 0.1;

		public static final double coralVelocityKP = 0.3;
		public static final double coralVelocityKI = 0.0;
		public static final double coralVelocityKD = 0.0;
		public static final double coralVelocityKS = 4.9;

		public static final double coralPositionKP = 0.75;
		public static final double coralPositionKI = 0.0;
		public static final double coralPositionKD = 0.0;

		public static final double algaeKP = 0.5;
		public static final double algaeKI = 0.0;
		public static final double algaeKD = 0.0;
		public static final double algaeKS = 0.0;

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

		public static TalonFXSConfiguration getCoralMotorConfiguration() {

			m_CoralMotorConfig.MotorOutput.Inverted = coralMotorInvert;
			m_CoralMotorConfig.MotorOutput.NeutralMode = coralMotorNuetralMode;

			m_CoralMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
			m_CoralMotorConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
			m_CoralMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

			// Current Limiting
			m_CoralMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
			m_CoralMotorConfig.CurrentLimits.SupplyCurrentLimit = coralSupplyCurrentLimit.in(Amps);
			m_CoralMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
					coralCurrentThresholdTime.in(Seconds);
			m_CoralMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
					coralCurrentLowerThreshold.in(Amps);

			m_CoralMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
			m_CoralMotorConfig.CurrentLimits.StatorCurrentLimit = coralStatorCurrentLimit.in(Amps);

			// PID Config
			m_CoralMotorConfig.Slot0.kP = coralVelocityKP;
			m_CoralMotorConfig.Slot0.kI = coralVelocityKI;
			m_CoralMotorConfig.Slot0.kD = coralVelocityKD;
			m_CoralMotorConfig.Slot0.kS = coralVelocityKS;

			m_CoralMotorConfig.Slot1.kP = coralPositionKP;
			m_CoralMotorConfig.Slot1.kI = coralPositionKI;
			m_CoralMotorConfig.Slot1.kD = coralPositionKD;

			// Open and Closed Loop Ramping
			m_CoralMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
			m_CoralMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

			m_CoralMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
			m_CoralMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;

			m_CoralMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;
			m_CoralMotorConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

			return m_CoralMotorConfig;
		}

		public static TalonFXConfiguration getAlgaeMotorConfiguration() {
			m_AlgaeMotorConfig.MotorOutput.Inverted = algaeMotorInvert;
			m_AlgaeMotorConfig.MotorOutput.NeutralMode = algaeMotorNuetralMode;

			// Current Limiting
			m_AlgaeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
			m_AlgaeMotorConfig.CurrentLimits.SupplyCurrentLimit = algaeSupplyCurrentLimit.in(Amps);
			m_AlgaeMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
					algaeCurrentThresholdTime.in(Seconds);
			m_AlgaeMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
					algaeLowerCurrentThreshold.in(Amps);

			m_AlgaeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
			m_AlgaeMotorConfig.CurrentLimits.StatorCurrentLimit = algaeStatorCurrentLimit.in(Amps);
			m_AlgaeMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
					algaeStatorCurrentLimitForward.in(Amps);
			m_AlgaeMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
					algaeStatorCurrentLimitReverse.in(Amps);

			// PID Config
			m_AlgaeMotorConfig.Slot0.kP = algaeKP;
			m_AlgaeMotorConfig.Slot0.kI = algaeKI;
			m_AlgaeMotorConfig.Slot0.kD = algaeKD;
			m_AlgaeMotorConfig.Slot0.kS = algaeKS;

			m_AlgaeMotorConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

			return m_AlgaeMotorConfig;
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
