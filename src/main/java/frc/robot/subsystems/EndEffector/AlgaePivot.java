// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.EndEffectorConstants.*;
import static frc.robot.Constants.EndEffectorConstants.MotorConfigs.*;
import static frc.robot.Constants.RobotConstants.*;

import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;
import frc.robot.state.RobotStates;

public class AlgaePivot extends SubsystemBase {

	// list of possible states with converts to absolute encoder rotations
	public static enum PivotStates {
		RETRACTED(pivotHomePosition),
		GROUNDINTAKE(pivotGroundIntakePosition),
		DEALGAEFY(pivotDeAlgifyPosition),
		PROCESSOR(pivotProcessorPosition);

		public final Angle Rotations;

		private PivotStates(Angle rotations) {
			this.Rotations = rotations;
		}
	}

	private PivotStates m_PivotState;

	private TalonFX m_AlgaePivotMotor;

	// public final TunableNumber homePosition;
	// public final TunableNumber groundIntakePosition;
	// public final TunableNumber deAlgifyPosition;
	// public final TunableNumber processorPosition;
	// public final TunableNumber netPosition;

	private final MotionMagicExpoTorqueCurrentFOC pivotRequest =
			new MotionMagicExpoTorqueCurrentFOC(Rotations.of(0));

	private final DutyCycleEncoder m_absoluteEncoder;

	public AlgaePivot() {
		// initialize motors
		m_AlgaePivotMotor = new TalonFX(pivotCanID, superstructureCANBusName);
		m_AlgaePivotMotor.getConfigurator().apply(getPivotConfiguration());
		m_PivotState = PivotStates.RETRACTED;

		pivotRequest.UpdateFreqHz = 0;
		pivotRequest.UseTimesync = true;

		m_absoluteEncoder =
				new DutyCycleEncoder(algaePivotEncoderPort, 1, algaePivotZeroPoint.in(Rotations));
		// reset position after a short delay
		Timer.delay(5);
		m_AlgaePivotMotor.setPosition(
				(getAbsolutePosition() - pivotEncoderOffset.in(Rotations)) * pivotMotorGearRatio);

		// homePosition = new TunableNumber("Algae Pivot/Home Position",
		// pivotHomePosition.in(Rotations));
		// groundIntakePosition =
		//     new TunableNumber(
		//         "Algae Pivot/Ground Intake Position", pivotGroundIntakePosition.in(Rotations));
		// deAlgifyPosition =
		//     new TunableNumber("Algae Pivot/DeAlgify Position", pivotDeAlgifyPosition.in(Rotations));
		// processorPosition =
		//     new TunableNumber("Algae Pivot/Processor Position",
		// pivotProcessorPosition.in(Rotations));
		// netPosition = new TunableNumber("Algae Pivot/Net Position", pivotNetPosition.in(Rotations));
	}

	/** set algae pivot state */
	public void setState(PivotStates state) {
		m_PivotState = state;
	}

	/** return algae pivot state */
	@Logged(key = "Pivot State", importance = Importance.CRITICAL)
	public PivotStates getState() {
		return m_PivotState;
	}

	/** return whether through-bore encoder is at position */
	public boolean isAtPositionAbsolute(double absolutePosition) {
		return Math.abs(absolutePosition - m_absoluteEncoder.get()) < algaePivotTolerance;
	}

	/** return whether motor encoder is at postion */
	public boolean isAtPosition(PivotStates state) {
		return Math.abs(
						state.Rotations.in(Rotations) - m_AlgaePivotMotor.getPosition().getValue().in(Rotations))
				< algaePivotTolerance;
	}

	/** reset motor encoder */
	public void resetPivotMotor(Angle rotations) {
		m_AlgaePivotMotor.setPosition(rotations);
	}

	/** return through-bore encoder position */
	public double getAbsolutePosition() {
		return m_absoluteEncoder.get();
	}

	/** return motor encoder position */
	public double getPosition() {
		return m_AlgaePivotMotor.getPosition().refresh().getValue().in(Rotations);
	}

	// set state and hold (persistent)
	public Command setStateCommand(PivotStates state) {
		return new InstantCommand(() -> setState(state), this).repeatedly();
	}

	// ends when pivot is fully retracted
	public Command retractCommand() {
		return new InstantCommand(() -> setState(PivotStates.RETRACTED), this)
				.repeatedly()
				.until(() -> isAtPosition(PivotStates.RETRACTED));
	}

	// set state and hold until algae is detected
	public Command waitForIntakeCommand(PivotStates state) {
		return new InstantCommand(() -> setState(state), this).repeatedly().until(RobotStates.hasAlgae);
	}

	@Override
	public void periodic() {
		// set output with position of current state
		m_AlgaePivotMotor.setControl(pivotRequest.withPosition(m_PivotState.Rotations));
	}
}