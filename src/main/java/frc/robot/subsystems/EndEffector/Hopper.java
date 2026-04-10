// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.IntakeIndexConstants.IndexerConstants.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;
import frc.lib.util.PeddieBounds;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;
import frc.robot.Constants.IntakeIndexConstants.IndexerConstants.MotorConfigs;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterTurretConstants.ShooterConstants;
import frc.robot.state.RobotStates;

public class Hopper extends SubsystemBase {

	public enum HopperStates {
		DEFAULT(defaultHopperSpeed),
		SHOOTING(runningHopperSpeed),
		REVERSING(reverseHopperSpeed);

		public final AngularVelocity hopperSpeed;

		private HopperStates(AngularVelocity hopperSpeed) {
			this.hopperSpeed = hopperSpeed;
		}
	}

	private final TalonFX m_HopperMotor;
	private final TalonFX m_ParallelMotor;

	private final VelocityTorqueCurrentFOC m_HopperRequest = new VelocityTorqueCurrentFOC(0);
	private final VelocityTorqueCurrentFOC m_ParallelRollerRequest = new VelocityTorqueCurrentFOC(0);

	private final Slot0Configs hopperConfigs = new Slot0Configs();
	private TunableNumber hopperKp;
	private TunableNumber hopperKd;
	private TunableNumber hopperKs;

	private final Slot0Configs parallelConfigs = new Slot0Configs();
	private TunableNumber parallelKp;
	private TunableNumber parallelKd;
	private TunableNumber parallelKs;
	private TunableNumber parallelKv;
	private TunableNumber parallelSpeed;

	private HopperStates m_HopperState;

	private boolean runParallel = false;
	private boolean   parallelRunningForward = false;
	private double timeAtChangeSeconds = 0;
	private Debouncer parallelJamDebouncer =
			new Debouncer(timeOfBackwardParallel.in(Seconds), Debouncer.DebounceType.kFalling);

	private AngularVelocity currentHopperVelocity = RotationsPerSecond.of(0);
	private AngularVelocity currentParallelVelocity = RotationsPerSecond.of(0);
	private AngularVelocity currentIndexerVelocity = RotationsPerSecond.of(0);

	private boolean parllelJamming = false;

	/** Creates a new Hopper. */
	public Hopper() {
		m_HopperMotor = new TalonFX(hopperMotorCanID, superstructureCANBusName);
		m_ParallelMotor = new TalonFX(parallelMotorCanID, superstructureCANBusName);
		m_HopperState = HopperStates.DEFAULT;

		m_HopperMotor.getConfigurator().apply(MotorConfigs.getHopperMotorConfig());
		m_ParallelMotor.getConfigurator().apply(MotorConfigs.getParallelMotorConfig());

		m_HopperRequest.UpdateFreqHz = 50;
		m_HopperRequest.UseTimesync = false;
		m_ParallelRollerRequest.UpdateFreqHz = 50;
		m_ParallelRollerRequest.UseTimesync = false;

		hopperConfigs
				.withKP(MotorConfigs.hopperVelocityKP)
				.withKD(MotorConfigs.hopperVelocityKD)
				.withKS(MotorConfigs.hopperVelocityKS);

		parallelConfigs
				.withKP(MotorConfigs.parallelVelocityKP)
				.withKD(MotorConfigs.parallelVelocityKD)
				.withKS(MotorConfigs.parallelVelocityKS)
				.withKV(MotorConfigs.parallelVelocityKV);

		initTunables();
	}

	@Logged(key = "Hopper Velocity", importance = Importance.DEBUG)
	public double getHopperVelocityRPS() {
		return currentHopperVelocity.in(RotationsPerSecond);
	}

	@Logged(key = "Parallel Roller Velocity", importance = Importance.DEBUG)
	public double getParallelVelocityRPS() {
		return currentParallelVelocity.in(RotationsPerSecond);
	}

	@Logged(key = "Parallel Roller Current", importance = Importance.DEBUG)
	public double getParallelCurrent() {
		return m_ParallelMotor.getStatorCurrent(true).getValueAsDouble();
	}

	@Logged(key = "Hopper State", importance = Importance.DEBUG)
	public HopperStates getHopperState() {
		return m_HopperState;
	}

	public void setState(HopperStates state) {
		m_HopperState = state;
	}

	public Command setStateCommand(HopperStates state) {
		return new InstantCommand(() -> setState(state), this)
				.repeatedly()
				.finallyDo(() -> setState(HopperStates.DEFAULT));
	}

	public Command setStateCommandPersistent(HopperStates state) {
		return new InstantCommand(() -> setState(state), this);
	}

	public void updateCache() {
		currentHopperVelocity = m_HopperMotor.getVelocity(true).getValue();
		currentParallelVelocity = m_ParallelMotor.getVelocity(true).getValue();
	}

	public boolean isParallelJammed() {
		return parllelJamming;
	}

	@Override
	public void periodic() {
		updateCache();
		updateTunables();

		parllelJamming = parallelJamDebouncer.calculate(getParallelCurrent() < parallelJamCurrent.in(Amps));

		runParallel = true;
				// runParallel
				// 		? !RobotStates.indexerRunning.getAsBoolean()
				// 		: RobotStates.indexerAtSpeed.getAsBoolean();

		Pose2d robotPose = RobotStates.robotPose.get();
		runParallel =
				runParallel
						&& robotPose
										.getTranslation()
										.getDistance(
												PeddieBounds.getShootingTargetPose(robotPose)
														.getTranslation()
														.toTranslation2d())
								> ShooterConstants.minShootingDistance.in(Meters);
		boolean parallelRunningForward2 = true;
		if (m_HopperState == HopperStates.SHOOTING && runParallel) {
			if (parallelRunningForward) {
				if (Timer.getFPGATimestamp() - timeAtChangeSeconds > timeOfForwardParallel.in(Seconds)) {
					parallelRunningForward = false;
					timeAtChangeSeconds = Timer.getFPGATimestamp();
				}
			} else {
				if (Timer.getFPGATimestamp() - timeAtChangeSeconds > timeOfBackwardParallel.in(Seconds)) {
					parallelRunningForward = true;
					timeAtChangeSeconds = Timer.getFPGATimestamp();
				}
			}


			m_ParallelMotor.setControl(
					m_ParallelRollerRequest.withVelocity(
							parllelJamming ? runningParallelSpeed : reverseParallelSpeed));
		} else {
			m_ParallelMotor.setControl(m_ParallelRollerRequest.withVelocity(defaultParallelSpeed));
		}

		if (m_HopperState.hopperSpeed.baseUnitMagnitude() == 0) {
			if (RobotStates.isIntakeDown.getAsBoolean()) {
				m_HopperMotor.setControl(m_HopperRequest.withVelocity(runningHopperSpeed));
			} else {
				m_HopperMotor.setControl(new CoastOut());
			}
		} else {
			if (m_HopperState == HopperStates.SHOOTING && runParallel) {
				m_HopperMotor.setControl(
						m_HopperRequest.withVelocity(
								parllelJamming
										? m_HopperState.hopperSpeed
										: m_HopperState.hopperSpeed.unaryMinus()));
			} else {
				m_HopperMotor.setControl(m_HopperRequest.withVelocity(m_HopperState.hopperSpeed));
			}
		}
	}

	public void initTunables() {
		if (RobotConstants.TuningModes.tuneHopper) {
			hopperKp = new TunableNumber("Hopper/Kp", MotorConfigs.hopperVelocityKP);
			hopperKd = new TunableNumber("Hopper/Kd", MotorConfigs.hopperVelocityKD);
			hopperKs = new TunableNumber("Hopper/Ks", MotorConfigs.hopperVelocityKS);
		}

		if (RobotConstants.TuningModes.tuneParallel) {
			parallelKp = new TunableNumber("Parallel/Kp", MotorConfigs.parallelVelocityKP);
			parallelKd = new TunableNumber("Parallel/Kd", MotorConfigs.parallelVelocityKD);
			parallelKs = new TunableNumber("Parallel/Ks", MotorConfigs.parallelVelocityKS);
			parallelKv = new TunableNumber("Parallel/Kv", MotorConfigs.parallelVelocityKV);
			parallelSpeed =
					new TunableNumber("Parallel/Speed", runningParallelSpeed.in(RotationsPerSecond));
		}
	}

	public void updateTunables() {
		if (RobotConstants.TuningModes.tuneHopper
				&& (hopperKp.getNumber() != hopperConfigs.kP
						|| hopperKd.getNumber() != hopperConfigs.kD
						|| hopperKs.getNumber() != hopperConfigs.kS)) {
			hopperConfigs
					.withKP(hopperKp.getNumber())
					.withKD(hopperKd.getNumber())
					.withKS(hopperKs.getNumber());
			m_HopperMotor.getConfigurator().apply(hopperConfigs);
		}

		if (RobotConstants.TuningModes.tuneParallel
				&& (parallelKp.getNumber() != parallelConfigs.kP
						|| parallelKd.getNumber() != parallelConfigs.kD
						|| parallelKs.getNumber() != parallelConfigs.kS
						|| parallelKv.getNumber() != parallelConfigs.kV)) {
			parallelConfigs
					.withKP(parallelKp.getNumber())
					.withKD(parallelKd.getNumber())
					.withKS(parallelKs.getNumber())
					.withKV(parallelKv.getNumber());
			m_ParallelMotor.getConfigurator().apply(parallelConfigs);
		}
	}
}
