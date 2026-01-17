// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;

import static edu.wpi.first.units.Units.Rotations;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Time;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

/** Add your docs here. */
public class EndEffectorConstants {

    public static final int intakeMotorCanID = 0; // Kraken X44
	public static final int pivotCanID = 0; // Kraken X60
    
	public static final int intakeEncoderPort = 0;
	public static final int pivotEncoderPort = 0;

    public static final Angle pivotZeroPoint = Rotations.of(0);

    public static final class MotorConfigs {

        public static final TalonFXConfiguration m_PivotConfig = new TalonFXConfiguration();
        public static final InvertedValue pivotInvert = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue pivotNeutralMode = NeutralModeValue.Brake;

        public static final Frequency timeSyncFreq = Hertz.of(250);

        // Pivot current limits
		public static final Current pivotSupplyCurrentLimit = Amps.of(40);
		public static final Current pivotCurrentLowerThreshold = Amps.of(30);

		public static final Current pivotStatorCurrentLimit = Amps.of(60);
		public static final Current pivotStatorCurrentLimitForward = Amps.of(60);
		public static final Current pivotStatorCurrentLimitReverse = Amps.of(-60);

		public static final Angle pivotForwardSoftLimit = Rotations.of(12.5);
		public static final Angle pivotReverseSoftLimit = Rotations.of(-3);

		public static final Time pivotCurrentThresholdTime = Seconds.of(0.50);

        //PID
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
