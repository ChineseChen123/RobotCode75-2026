// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import static frc.robot.Constants.FieldConstants.algaeHeights;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.PeddieBounds;
import frc.robot.commands.EndEffector.SetElevatorPosition;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Drivetrain.controllers.ChezyController;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaeIntake.AlgaeStates;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.AlgaePivot.PivotStates;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDealgaefy extends SequentialCommandGroup {
	/** Creates a new AutoDealgaefy. */
	public AutoDealgaefy(
			Swerve swerve,
			Elevator elevator,
			AlgaeIntake intake,
			AlgaePivot pivot,
			ChezyController chezyController) {
		addRequirements(swerve, elevator, intake, pivot);
		FieldElement elem = PeddieBounds.nearestElement(swerve.getPose());
		if (!FieldPose.fieldElementIsReef(elem)) {
			return;
		}
		ElevatorPositions elevatorHeight = algaeHeights.get(elem.toString());
		addCommands(
				new ParallelCommandGroup(
						// new ChezyPose(swerve, chezyController, new
						// FieldPose(DriverStation.getAlliance().get(), elem, Offset.MID), false),
						new SetElevatorPosition(elevator, elevatorHeight, true, false)),
				new ParallelCommandGroup(
								new InstantCommand(
												() -> {
													intake.setState(AlgaeStates.INTAKING);
													pivot.setState(PivotStates.DEALGAEFY);
												},
												intake,
												pivot)
										.repeatedly()
										.until(() -> intake.getState() == AlgaeStates.HOLD),
								new SetElevatorPosition(elevator, elevatorHeight, true, false))
						.until(() -> intake.getState() == AlgaeStates.HOLD),
				new ParallelCommandGroup(
						new InstantCommand(
								() -> {
									swerve.setRobotRelative(new ChassisSpeeds(-0.25, 0, 0));
								}),
						new WaitCommand(0.5),
						new SetElevatorPosition(elevator, elevatorHeight, true, false)),
				new InstantCommand(
								() -> {
									pivot.setState(PivotStates.RETRACTED);
									elevator.setPosition(ElevatorPositions.HOME, false);
								})
						.repeatedly()
						.until(() -> pivot.isAtPosition(PivotStates.RETRACTED)));
	}
}
