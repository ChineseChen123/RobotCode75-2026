// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.CoralIntake.CoralStates;

public class IntakeCoral extends Command {
	private final CoralIntake m_coralIntake;

	/** Creates a new IntakeCoral. */
	public IntakeCoral(CoralIntake coralIntake) {
		m_coralIntake = coralIntake;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_coralIntake);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_coralIntake.setState(CoralStates.INTAKING);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_coralIntake.setState(CoralStates.DEFAULT);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_coralIntake.hasCoral();
		// return m_coralIntake.getState() == CoralStates.HASGAMEPIECE;
		// return false;
	}
}
