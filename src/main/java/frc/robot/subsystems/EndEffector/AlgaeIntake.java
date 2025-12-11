// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.EndEffectorConstants.*;
import static frc.robot.Constants.EndEffectorConstants.MotorConfigs.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;
import frc.robot.subsystems.LidarDistanceSensor;

public class AlgaeIntake extends SubsystemBase {

	// list of possible states
	public static enum AlgaeStates {
		DEFAULT,
		INTAKING,
		HOLD,
		SCORING
	}

	private AlgaeStates m_AlgaeIntakeState;

	private TalonFX m_AlgaeMotor;

	private final LidarDistanceSensor m_AlgaeDetector;

	// intake request
	private final VelocityTorqueCurrentFOC algaeRequest =
			new VelocityTorqueCurrentFOC(RotationsPerSecond.of(0));
	// default request
	private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(Amps.of(0));

	public AlgaeIntake() {
		// initialize motors and sensors
		m_AlgaeMotor = new TalonFX(algaeMotorCanID, superstructureCANBusName);
		m_AlgaeMotor.getConfigurator().apply(getAlgaeMotorConfiguration());
		m_AlgaeIntakeState = AlgaeStates.DEFAULT;
		m_AlgaeDetector = new LidarDistanceSensor(Inches.of(2.75));

		algaeRequest.UpdateFreqHz = 0;
		algaeRequest.UseTimesync = true;

		currentOut.UpdateFreqHz = 0;
		currentOut.UseTimesync = true;
	}

	/** set state (persistent) except when holding */
	public void setState(AlgaeStates state) {
		if (m_AlgaeIntakeState == AlgaeStates.HOLD && state != AlgaeStates.SCORING) {
			m_AlgaeIntakeState = AlgaeStates.HOLD;
		} else {
			m_AlgaeIntakeState = state;
		}
	}

	/** return whether algae is detected */
	@Logged(key = "Has Algae", importance = Importance.CRITICAL)
	public boolean hasAlgae() {
		return m_AlgaeDetector.belowThreshold();
	}

	/** get state */
	@Logged(key = "Algae Intake State", importance = Importance.CRITICAL)
	public AlgaeStates getState() {
		return m_AlgaeIntakeState;
	}

	/** set state until interrupted (non-persistent) */
	public Command setStateCommand(AlgaeStates state) {
		return new InstantCommand(() -> setState(state), this)
				.repeatedly()
				.finallyDo(() -> setState(AlgaeStates.DEFAULT));
	}

	/** intake and set next state accordingly */
	public Command intakeCommand(Trigger bind) {
		return new InstantCommand(() -> setState(AlgaeStates.INTAKING), this)
				.repeatedly()
				.until(bind.negate().or(this::hasAlgae))
				.finallyDo(() -> setState(hasAlgae() ? AlgaeStates.HOLD : AlgaeStates.DEFAULT));
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("Has Algae", hasAlgae());

		// stop motor if no algae and not intaking
		if (!hasAlgae() && m_AlgaeIntakeState != AlgaeStates.INTAKING) {
			m_AlgaeIntakeState = AlgaeStates.DEFAULT;
		}

		// set output based on state
		switch (m_AlgaeIntakeState) {
			case INTAKING -> {
				m_AlgaeMotor.setControl(algaeRequest.withVelocity(algaeIntakeSpeed));
			}
			case HOLD -> {
				// set the velocity control to a lower value to hold the algae in
				m_AlgaeMotor.setControl(algaeRequest.withVelocity(algaeHoldSpeed));
			}
			case SCORING -> {
				m_AlgaeMotor.setControl(algaeRequest.withVelocity(algaeScoreSpeed));
			}
			case DEFAULT -> {
				// set a DEFAULT state for when there is no algae and we are not intaking anything
				m_AlgaeMotor.setControl(currentOut.withOutput(Amps.of(0)));
			}
		}
	}
}
