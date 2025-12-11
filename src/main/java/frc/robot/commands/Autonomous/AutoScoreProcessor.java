// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import frc.lib.util.PeddieBounds;
import frc.robot.commands.Drivetrain.ChezyPose;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaeIntake.AlgaeStates;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.AlgaePivot.PivotStates;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScoreProcessor extends SequentialCommandGroup {
	public AutoScoreProcessor(Swerve swerve, AlgaeIntake intake, AlgaePivot pivot) {
		addRequirements(swerve, intake, pivot);
		Pose2d poseToDrive =
				PeddieBounds.getNearestFieldPose2d(
						swerve.getPose(),
						new FieldPose(DriverStation.getAlliance().get(), FieldElement.P, Offset.MID));
		addCommands(
				new ChezyPose(poseToDrive, false),
				new InstantCommand(
								() -> {
									intake.setState(AlgaeStates.SCORING);
									pivot.setState(PivotStates.RETRACTED);
								})
						.repeatedly()
						.until(() -> !intake.hasAlgae()),
				new InstantCommand(
						() -> {
							intake.setState(AlgaeStates.DEFAULT);
						}));
	}
}
