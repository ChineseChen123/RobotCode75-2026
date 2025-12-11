// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector.Algae;

import static frc.robot.Constants.EndEffectorConstants.pivotDeAlgaefyDelay;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.EndEffector.SetElevatorPosition;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaeIntake.AlgaeStates;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.AlgaePivot.PivotStates;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeAlgaefy extends SequentialCommandGroup {
	/** Creates a new DeAlgaefy. */
	// TODO: use a conditional command
	public DeAlgaefy(
			Elevator elevator, AlgaeIntake algaeIntake, AlgaePivot algaePivot, boolean isL2) {
		if (isL2) { // temporary fix until we get april tags
			/*
			 * first move elev to position
			 *  parallel:
			 *    {
			 *    spin wheels
			 *    extend pivot
			 *      UNTIL: limit switch is pressed (button release)
			 *    }
			 *    {
			 *      HOLD ELEV POSIITON
			 *    }
			 *    THEN:
			 *     Sequence:
			 *      switch state to has game peice
			 *      retract pivot
			 *  THEN
			 *    lower elev
			 */
			addRequirements(elevator);
			addCommands(
					new SetElevatorPosition(elevator, ElevatorPositions.L1, true, false),
					new ParallelCommandGroup(
									new InstantCommand(
													() -> {
														algaeIntake.setState(AlgaeStates.INTAKING);
														algaePivot.setState(PivotStates.DEALGAEFY);
													},
													algaeIntake,
													algaePivot)
											.repeatedly()
											.until(() -> algaeIntake.getState() == AlgaeStates.HOLD),
									new SetElevatorPosition(elevator, ElevatorPositions.L1, true, false))
							.until(() -> algaeIntake.getState() == AlgaeStates.HOLD),
					new WaitCommand(pivotDeAlgaefyDelay),
					new InstantCommand(() -> algaePivot.setState(PivotStates.RETRACTED))
							.repeatedly()
							.until(() -> algaePivot.isAtPosition(PivotStates.RETRACTED)));
		} else {
			/* L3 Algae Intake */
			addCommands(
					new SetElevatorPosition(elevator, ElevatorPositions.L3, true, false),
					new ParallelCommandGroup(
									new InstantCommand(
													() -> {
														algaeIntake.setState(AlgaeStates.INTAKING);
														algaePivot.setState(PivotStates.DEALGAEFY);
													},
													algaeIntake,
													algaePivot)
											.repeatedly()
											.until(() -> algaeIntake.getState() == AlgaeStates.HOLD),
									new SetElevatorPosition(elevator, ElevatorPositions.L3, true, false))
							.until(() -> algaeIntake.getState() == AlgaeStates.HOLD),
					new WaitCommand(pivotDeAlgaefyDelay),
					new InstantCommand(() -> algaePivot.setState(PivotStates.RETRACTED))
							.repeatedly()
							.until(() -> algaePivot.isAtPosition(PivotStates.RETRACTED)));
		}
	}
}
