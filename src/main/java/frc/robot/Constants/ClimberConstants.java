// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

public class ClimberConstants {
	// from the perspective of looking from the back of the robot forwards
	public static final int climberMotor1CANID = 61; // left
	public static final int climberMotor2CANID = 62; // right

	// position from home
	public static final Angle stowedPosition = Rotations.of(0);
	public static final Angle upPosition = Rotations.of(0);
	public static final Angle climbedPosition = Rotations.of(0);

	public static final Angle climberTolerance = Rotations.of(1);

	public static final class MotorConfigs {

		public static final InvertedValue climberMotorInverted = InvertedValue.Clockwise_Positive;

		// takes 0.25 seconds to go from 0-100% current output
		public static final Time closedLoopRamp = Seconds.of(0.25);

		// current limits
		public static final Current statorCurrentLimit = Amps.of(60);
		public static final Current supplyCurrentLimit = Amps.of(40);

		// set current limit to 30 amps if supply current limit is exceeded for more than 0.5 seconds
		public static final Current supplyCurrentLowerLimit = Amps.of(30);
		public static final Time supplyCurrentLowerTime = Seconds.of(0.5);

		public static final Angle forwardLimit = Rotations.of(0);
		public static final Angle reverseLimit = Rotations.of(0);

		public static final Frequency timeSyncFreq = Hertz.of(250);

		// PIDs
		public static final double kA = 0; // current per unit of acceleration
		public static final double kG = 0; // current to overcome gravity
		public static final double kS = 0; // current to overcome static friction
		public static final double kV = 0; // current per unit of requested velocity
		public static final double kP = 0;
		public static final double kI = 0;
		public static final double kD = 0;

		public static final Current torqueForwardCurrentLimit = Amps.of(80);
		public static final Current torqueReverseCurrentLimit = Amps.of(80);

		public static TalonFXConfiguration getClimberMotorConfig() {
			TalonFXConfiguration m_ClimberMotorConfig = new TalonFXConfiguration();

			m_ClimberMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp.in(Seconds);

			// Current Limits
			m_ClimberMotorConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit.in(Amps);
			m_ClimberMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

			m_ClimberMotorConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit.in(Amps);
			m_ClimberMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
			m_ClimberMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = supplyCurrentLowerLimit.in(Amps);
			m_ClimberMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
					supplyCurrentLowerTime.in(Seconds);

			m_ClimberMotorConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);
			m_ClimberMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
			m_ClimberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

			// PID Configs
			m_ClimberMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
			m_ClimberMotorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
			m_ClimberMotorConfig.Slot0.kA = kA; // tune third
			m_ClimberMotorConfig.Slot0.kG = kG; // tune first
			m_ClimberMotorConfig.Slot0.kS = kS; // tune second
			m_ClimberMotorConfig.Slot0.kV = kV; // tune third
			m_ClimberMotorConfig.Slot0.kP = kP; // tune fourth
			m_ClimberMotorConfig.Slot0.kI = kI; // tune only if needed
			m_ClimberMotorConfig.Slot0.kD = kD; // tune fifth

			m_ClimberMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
					torqueForwardCurrentLimit.in(Amps);
			m_ClimberMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
					torqueReverseCurrentLimit.in(Amps);

			m_ClimberMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
			m_ClimberMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
					forwardLimit.in(Rotations);
			m_ClimberMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
			m_ClimberMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
					reverseLimit.in(Rotations);

			return m_ClimberMotorConfig;
		}
	}
}
