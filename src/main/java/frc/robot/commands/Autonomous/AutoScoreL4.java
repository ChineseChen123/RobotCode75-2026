// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.EndEffector.Coral.IntakeCoral;
import frc.robot.commands.EndEffector.Coral.ScoreCoral;
import frc.robot.commands.EndEffector.SetElevatorPosition;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScoreL4 extends SequentialCommandGroup {
	public AutoScoreL4(Elevator elevator, CoralIntake coralIntake) {
		addRequirements(elevator, coralIntake);
		addCommands(
				new IntakeCoral(coralIntake).onlyIf(() -> !coralIntake.hasCoral()),
				new SetElevatorPosition(elevator, ElevatorPositions.L4, false, false),
				new ParallelDeadlineGroup(
						new ScoreCoral(coralIntake, elevator),
						new SetElevatorPosition(elevator, ElevatorPositions.L4, false, false)),
				new SetElevatorPosition(elevator, ElevatorPositions.HOME, false, false)
				// new InstantCommand(() -> System.out.println("done"))
				);
	}
}
