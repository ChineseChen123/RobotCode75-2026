// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChoppedPreraiseElevator extends Command {
	private final Swerve m_Swerve;
	private final Elevator m_Elevator;

	/** Creates a new ChoppedPreraiseElevator. */
	public ChoppedPreraiseElevator(Swerve swerve, Elevator elevator) {
		m_Swerve = swerve;
		m_Elevator = elevator;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_Elevator);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_Elevator.setPosition(ElevatorPositions.HOME, false);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		ChassisSpeeds speeds = m_Swerve.getChassisSpeeds();
		if (Math.abs(speeds.vyMetersPerSecond) > 1) {
			m_Elevator.setPosition(ElevatorPositions.HOME, false);
		} else if (Math.abs(speeds.vyMetersPerSecond) > 0.5) {
			m_Elevator.setPosition(ElevatorPositions.L2, false);
		} else if (Math.abs(speeds.vyMetersPerSecond) > 0.25) {
			m_Elevator.setPosition(ElevatorPositions.L3, false);
		} else {
			m_Elevator.setPosition(ElevatorPositions.L4, false);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
