// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import static frc.robot.Constants.EndEffectorConstants.coralScoreDelay;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.EndEffector.Coral.IntakeCoral;
import frc.robot.commands.EndEffector.Coral.ScoreCoral;
import frc.robot.commands.EndEffector.SetElevatorPosition;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScoreL1 extends SequentialCommandGroup {
	public AutoScoreL1(Swerve swerve, Elevator elevator, CoralIntake coralIntake) {
		addRequirements(swerve, elevator, coralIntake);
		addCommands(
				new IntakeCoral(coralIntake),
				new ParallelCommandGroup(
						// align to branch color
						new SetElevatorPosition(elevator, ElevatorPositions.L1, false, false)),
				new ParallelCommandGroup(
						new ScoreCoral(coralIntake, elevator),
						new SetElevatorPosition(elevator, ElevatorPositions.L1, false, false)),
				new ParallelCommandGroup(
						new SetElevatorPosition(elevator, ElevatorPositions.L1, false, false),
						new WaitCommand(coralScoreDelay)),
				new SetElevatorPosition(elevator, ElevatorPositions.HOME, false, true));
	}
}
