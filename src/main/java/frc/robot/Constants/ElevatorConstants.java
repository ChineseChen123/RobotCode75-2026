// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

public class ElevatorConstants {
	// from the perspective of looking from the back of the robot forwards
	public static final int elevatorMotor1CANID = 41; // left
	public static final int elevatorMotor2CANID = 42; // right
	public static final int upperLimitPort = 0;
	public static final int lowerLimitPort = 1;
	public static final int backupLimitPort = 3;

	public static final Distance pulleyCircumference =
			Inches.of(1.751 * Math.PI); // circumference of 22 teeth #25 WCP sprocket
	public static final double mechanismToMotorRatio =
			5.0 + (1.0 / 3.0); // 3 motor rotations = 1 shaft rotation

	public static final Distance inchesPerRotation =
			Inches.of(
					pulleyCircumference.in(Inches)
							* 2
							/ mechanismToMotorRatio); // 1 motor rotation = 2.06 inches

	// position from home
	public static final Angle algaeRemovalOffset = Rotations.of(5.5); // should be around 5?
	public static final Angle l1Position = Rotations.of(6.6);
	public static final Angle l2Position = Rotations.of(6.6);
	public static final Angle l3Position = Rotations.of(14.375);
	public static final Angle l4Position = Rotations.of(25.8);
	public static final Angle netPosition = Rotations.of(26);
	public static final Angle homePosition = Rotations.of(0);
	public static final Angle processorPosition = Rotations.of(2);

	// velocity, acceleration, jerk
	public static final double[] MotionMagicProfileUp = {150, 170, 1200};
	public static final double[] MotionMagicProfileDown = {110, 90, 1000};

	public static final Angle elevatorTolerance = Rotations.of(0.7);

	public static final class MotorConfigs {

		// takes 0.25 seconds to go from 0-100% current output
		public static final Time closedLoopRamp = Seconds.of(0.25);

		// current limits
		public static final Current statorCurrentLimit = Amps.of(60);
		public static final Current supplyCurrentLimit = Amps.of(40);

		// set current limit to 30 amps if supply current limit is exceeded for more than 0.5 seconds
		public static final Current supplyCurrentLowerLimit = Amps.of(30);
		public static final Time supplyCurrentLowerTime = Seconds.of(0.5);

		public static final Angle forwardLimit = Rotations.of(26);
		public static final Angle reverseLimit = Rotations.of(0);

		public static final Frequency timeSyncFreq = Hertz.of(250);

		// PIDs
		public static final double kA = 0.3; // current per unit of acceleration
		public static final double kG = 20; // current to overcome gravity
		public static final double kS = 19; // current to overcome static friction
		public static final double kV = 0.12; // current per unit of requested velocity
		public static final double kP = 25;
		public static final double kI = 0;
		public static final double kD = 5;

		public static final Current torqueForwardCurrentLimit = Amps.of(100);
		public static final Current torqueReverseCurrentLimit = Amps.of(100);

		public static TalonFXConfiguration getElevatorMotorConfig() {
			TalonFXConfiguration m_ElevatorMotorConfig = new TalonFXConfiguration();

			m_ElevatorMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp.in(Seconds);

			// Current Limits
			m_ElevatorMotorConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit.in(Amps);
			m_ElevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

			m_ElevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit.in(Amps);
			m_ElevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
			m_ElevatorMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
					supplyCurrentLowerLimit.in(Amps);
			m_ElevatorMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
					supplyCurrentLowerTime.in(Seconds);

			m_ElevatorMotorConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);
			m_ElevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
			m_ElevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

			// PID Configs
			m_ElevatorMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
			m_ElevatorMotorConfig.Slot0.StaticFeedforwardSign =
					StaticFeedforwardSignValue.UseVelocitySign;
			m_ElevatorMotorConfig.Slot0.kA = kA; // tune third
			m_ElevatorMotorConfig.Slot0.kG = kG; // tune first
			m_ElevatorMotorConfig.Slot0.kS = kS; // tune second
			m_ElevatorMotorConfig.Slot0.kV = kV; // tune third
			m_ElevatorMotorConfig.Slot0.kP = kP; // tune fourth
			m_ElevatorMotorConfig.Slot0.kI = kI; // tune only if needed
			m_ElevatorMotorConfig.Slot0.kD = kD; // tune fifth

			m_ElevatorMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
					torqueForwardCurrentLimit.in(Amps);
			m_ElevatorMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
					torqueReverseCurrentLimit.in(Amps);

			m_ElevatorMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
			m_ElevatorMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
					forwardLimit.in(Rotations);
			m_ElevatorMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
			m_ElevatorMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
					reverseLimit.in(Rotations);

			return m_ElevatorMotorConfig;
		}
	}
}
