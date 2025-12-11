// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPosition extends Command {
	/** Creates a new SetElevatorPosition. */
	private final Elevator m_Elevator;

	private final ElevatorPositions m_Position;
	private final boolean m_IsAlgae;
	private final boolean m_LastAutoCommand;

	public SetElevatorPosition(
			Elevator elevator, ElevatorPositions position, boolean isAlgae, boolean lastAutoCommand) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_LastAutoCommand = lastAutoCommand;
		m_Elevator = elevator;
		m_Position = position;
		m_IsAlgae = isAlgae;
		addRequirements(m_Elevator);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_Elevator.setPosition(m_Position, m_IsAlgae);
		// System.out.println(m_Position.toString() + " " + Timer.getFPGATimestamp());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// System.out.println(m_Elevator.isBelowPosition(ElevatorPositions.L3, false));
		if (m_LastAutoCommand) {
			return m_Elevator.isBelowPosition(ElevatorPositions.L3, false);
		} else {
			return m_Elevator.isAtPosition(m_Position, m_IsAlgae);
		}
	}
}
