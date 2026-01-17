// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class EndEffectorConstants {

	public class Shooter {
		// Kraken X60s
		public static final int flywheelCanID = 0;
		public static final int rollerCanID = 0;

		public static final double flywheelGearRatio = 1;
		public static final double rollerGearRatio = 1;

		public static final Distance flywheelWheelDiameter = Meters.of(0);
		public static final Distance rollerWheelDiameter = Meters.of(0);

		public static final AngularVelocity defaultFlywheelSpeed = RotationsPerSecond.of(0);
		public static final AngularVelocity defaultRollerSpeed = RotationsPerSecond.of(0);

		public static final AngularVelocity reverseFlywheelSpeed = RotationsPerSecond.of(-5);
		public static final AngularVelocity reverseRollerSpeed = RotationsPerSecond.of(-5);

		public static final class MotorConfigs {

			public static final TalonFXConfiguration m_FlywheelConfig = new TalonFXConfiguration();
			public static final TalonFXConfiguration m_RollerConfig = new TalonFXConfiguration();

			// Neutral modes and inverts
			public static final InvertedValue flywheelInverted = InvertedValue.CounterClockwise_Positive;
			public static final InvertedValue rollerInverted = InvertedValue.Clockwise_Positive;

			public static final NeutralModeValue flywheelNeutralMode = NeutralModeValue.Coast;
			public static final NeutralModeValue rollerNeutralMode = NeutralModeValue.Coast;

			public static final Frequency timeSyncFreq = Hertz.of(250);

			public static final Time flywheelCurrentThresholdTime = Seconds.of(0.5);
			public static final Time rollerCurrentThresholdTime = Seconds.of(0.5);

			public static final Current flywheelSupplyCurrentLimit = Amps.of(40);
			public static final Current flywheelCurrentLowerThreshold = Amps.of(30);
			public static final Current flywheelStatorCurrentLimit = Amps.of(60);

			public static final Current rollerSupplyCurrentLimit = Amps.of(40);
			public static final Current rollerCurrentLowerThreshold = Amps.of(30);
			public static final Current rollerStatorCurrentLimit = Amps.of(60);

			// Torque PI
			public static final double openLoopRamp = 0.1;
			public static final double closedLoopRamp = 0.1;

			public static final double flywheelVelocityKP = 0.3;
			public static final double flywheelVelocityKI = 0.0;
			public static final double flywheelVelocityKD = 0.0;
			public static final double flywheelVelocityKS = 4.9;

			public static final double rollerVelocityKP = 0.3;
			public static final double rollerVelocityKI = 0.0;
			public static final double rollerVelocityKD = 0.0;
			public static final double rollerVelocityKS = 4.9;

			public static TalonFXConfiguration getFlywheelMotorConfiguration() {

				m_FlywheelConfig.MotorOutput.Inverted = flywheelInverted;
				m_FlywheelConfig.MotorOutput.NeutralMode = flywheelNeutralMode;

				// Current Limiting
				m_FlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
				m_FlywheelConfig.CurrentLimits.SupplyCurrentLimit = flywheelSupplyCurrentLimit.in(Amps);
				m_FlywheelConfig.CurrentLimits.SupplyCurrentLowerTime =
						flywheelCurrentThresholdTime.in(Seconds);
				m_FlywheelConfig.CurrentLimits.SupplyCurrentLowerLimit =
						flywheelCurrentLowerThreshold.in(Amps);

				m_FlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
				m_FlywheelConfig.CurrentLimits.StatorCurrentLimit = flywheelStatorCurrentLimit.in(Amps);

				// PID Config
				m_FlywheelConfig.Slot0.kP = flywheelVelocityKP;
				m_FlywheelConfig.Slot0.kI = flywheelVelocityKI;
				m_FlywheelConfig.Slot0.kD = flywheelVelocityKD;
				m_FlywheelConfig.Slot0.kS = flywheelVelocityKS;

				// Open and Closed Loop Ramping
				m_FlywheelConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
				m_FlywheelConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

				m_FlywheelConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
				m_FlywheelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;

				m_FlywheelConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;
				m_FlywheelConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

				return m_FlywheelConfig;
			}

			public static TalonFXConfiguration getRollerMotorConfiguration() {

				m_RollerConfig.MotorOutput.Inverted = rollerInverted;
				m_RollerConfig.MotorOutput.NeutralMode = rollerNeutralMode;

				// Current Limiting
				m_RollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
				m_RollerConfig.CurrentLimits.SupplyCurrentLimit = rollerSupplyCurrentLimit.in(Amps);
				m_RollerConfig.CurrentLimits.SupplyCurrentLowerTime =
						rollerCurrentThresholdTime.in(Seconds);
				m_RollerConfig.CurrentLimits.SupplyCurrentLowerLimit = rollerCurrentLowerThreshold.in(Amps);
				m_RollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
				m_RollerConfig.CurrentLimits.StatorCurrentLimit = rollerStatorCurrentLimit.in(Amps);

				// PID Config
				m_RollerConfig.Slot0.kP = rollerVelocityKP;
				m_RollerConfig.Slot0.kI = rollerVelocityKI;
				m_RollerConfig.Slot0.kD = rollerVelocityKD;
				m_RollerConfig.Slot0.kS = rollerVelocityKS;

				// Open and Closed Loop Ramping
				m_RollerConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
				m_RollerConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

				m_RollerConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
				m_RollerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;

				m_RollerConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;
				m_RollerConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

				return m_RollerConfig;
			}
		}
	}
}
