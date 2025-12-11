// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Drivetrain.YoloBranchAlign;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.CoralIntake.CoralStates;
import frc.robot.subsystems.EndEffector.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoL4WithFallback extends Command {
	/** Creates a new AutoL4WithFallback. */
	private final Swerve m_Swerve;

	private final Elevator m_Elevator;
	private final CoralIntake m_CoralIntake;

	private final Command command;

	double start_time;

	public AutoL4WithFallback(Swerve swerve, Elevator elevator, CoralIntake coralIntake) {
		m_Swerve = swerve;
		m_Elevator = elevator;
		m_CoralIntake = coralIntake;

		command =
				new ParallelCommandGroup(
						new AutoScoreL4(elevator, coralIntake), new YoloBranchAlign(m_Swerve, true));
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_Swerve, m_Elevator, m_CoralIntake);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		start_time = Timer.getFPGATimestamp();
		command.schedule();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (m_CoralIntake.getState() != CoralStates.INTAKING) {
			start_time = Timer.getFPGATimestamp();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		command.cancel();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {

		double time_since_start = Timer.getFPGATimestamp() - start_time;

		if (time_since_start >= 1.0) {
			return true;
		}

		return false;
	}
}
