// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.CoralIntake.CoralStates;
import frc.robot.subsystems.EndEffector.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoral extends Command {
	/** Creates a new ScoreCoral. */
	private final CoralIntake m_Intake;

	private final Elevator m_Elevator;

	public ScoreCoral(CoralIntake intake, Elevator elevator) {
		m_Elevator = elevator;
		m_Intake = intake;
		addRequirements(m_Intake);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_Intake.setState(CoralStates.SCOREL4);
		// System.out.println("Scoring " + Timer.getFPGATimestamp());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("scoring done");
		m_Intake.setState(CoralStates.DEFAULT);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return !m_Intake.hasCoral();
	}
}
