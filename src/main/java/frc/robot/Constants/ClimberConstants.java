package frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

public final class ClimberConstants {
	public static final double climberGearRatio = 39.6 / 1.0;
	// motor encoder setpoint when fully retracted
	public static final Angle climbPosition = Rotations.of(2.5);
	// motor encoder setpoint when fully deployed
	public static final Angle climbExtendPosition = Rotations.of(130);
	public static final double climbPositionTolerance = 0.5;
	
	public static final int limitPort = 8;
	public static final int climberMotor2CANID = 47;
	public static final int climberMotor1CANID = 46;
	public static final int climberEncoderPort = 6;

	public static final class MotorConfigs {
		public static final Time closedLoopRamp = Seconds.of(0.25);

		public static final Current statorCurrentLimit = Amps.of(60);
		public static final Current supplyCurrentLimit = Amps.of(40);
		// set current limit to 30 amps if supply current limit is exceeded for more than 0.5 seconds
		public static final Current supplyCurrentLowerLimit = Amps.of(30);
		public static final Time supplyCurrentLowerTime = Seconds.of(0.5);

		public static final Current statorForwardCurrentLimit = Amps.of(100);
		public static final Current statorReverseCurrentLimit = Amps.of(100);

		public static final Angle forwardSoftLimit = Rotations.of(104);
		public static final Angle reverseSoftLimit = Rotations.of(0.01);

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

		public static TalonFXConfiguration getClimberMotorConfig() {
			TalonFXConfiguration m_ClimberMotorConfig = new TalonFXConfiguration();

			m_ClimberMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp.in(Seconds);

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

			m_ClimberMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
			m_ClimberMotorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
			m_ClimberMotorConfig.Slot0.kA = kA; // tune third
			m_ClimberMotorConfig.Slot0.kG = kG; // tune first
			m_ClimberMotorConfig.Slot0.kS = kS; // tune second
			m_ClimberMotorConfig.Slot0.kV = kV; // tune third
			m_ClimberMotorConfig.Slot0.kP = kP; // tune fourth
			m_ClimberMotorConfig.Slot0.kI = kI; // tune only if needed
			m_ClimberMotorConfig.Slot0.kD = kD; // tune fifth

			m_ClimberMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
					statorForwardCurrentLimit.in(Amps);
			m_ClimberMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
					statorReverseCurrentLimit.in(Amps);

			m_ClimberMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
			m_ClimberMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
					forwardSoftLimit.in(Rotations);
			m_ClimberMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
			m_ClimberMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
					reverseSoftLimit.in(Rotations);

			return m_ClimberMotorConfig;
		}
	}
}
