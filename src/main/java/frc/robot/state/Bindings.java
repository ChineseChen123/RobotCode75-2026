package frc.robot.state;

import static frc.robot.Constants.IOConstants.oneDriver;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import frc.lib.util.PeddieBounds;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Swerve;

public class Bindings {

	/** binds commands to all actions */
	public void bind2Driver() {
		Swerve swerve = RobotContainer.getSwerve();

		// Swerve bindings
		swerve.setDefaultCommand(swerve.teleopSwerveCommand());
		RobotStates.actionResetGyro.whileTrue(swerve.resetHeadingCommand());
		if (!oneDriver) {
			RobotStates.actionXStance.whileTrue(swerve.xStanceCommand());
		}
		// RobotStates.actionRotateSimilarFace.whileTrue(swerve.similarFaceRotateCommand());
		// if (!oneDriver) {
		// 	RobotStates.actionHPRotate.whileTrue(swerve.hpRotateCommand());
		// }
		RobotStates.actionRobotRelative.onChange(
				new InstantCommand(() -> swerve.toggleFieldRelative()));

		// Algae Intake bindings
		// Intake if no algae is present
		// RobotStates.actionGroundAlgae
		// 		.and(RobotStates.hasAlgae.negate())
		// 		.whileTrue(algaeIntake.intakeCommand(RobotStates.actionGroundAlgae));
		// if (!oneDriver) {
		// 	// Score only if algae is present
		// 	RobotStates.actionScoreAlgae
		// 			.and(RobotStates.hasAlgae)
		// 			.whileTrue(algaeIntake.setStateCommand(AlgaeStates.SCORING));
		// }
		// // Intake from reef once elevator is at the correct level
		// RobotStates.actionDeAlgaefyL2
		// 		.and(RobotStates.elevatorAtL2Algae)
		// 		.whileTrue(algaeIntake.intakeCommand(RobotStates.actionDeAlgaefyL2));
		// RobotStates.actionDeAlgaefyL3
		// 		.and(RobotStates.elevatorAtL3Algae)
		// 		.whileTrue(algaeIntake.intakeCommand(RobotStates.actionDeAlgaefyL3));

		// // Algae Pivot bindings
		// algaePivot.setDefaultCommand(algaePivot.setStateCommand(PivotStates.RETRACTED));
		// if (!oneDriver) {
		//
		//	RobotStates.actionProcessorPivot.whileTrue(algaePivot.setStateCommand(PivotStates.PROCESSOR));
		// }
		// // Keep pivot down until elevator raises
		// RobotStates.actionGroundAlgae.whileTrue(
		// 		algaePivot.setStateCommand(PivotStates.GROUNDINTAKE).until(RobotStates.elevatorAtL1));
		// // Retract after elevator raises
		// RobotStates.actionGroundAlgae
		// 		.and(RobotStates.elevatorAtL1)
		// 		.whileTrue(algaePivot.retractCommand());
		// // Lower until algae detected, wait a set duration, then retract
		// RobotStates.actionDeAlgaefyL2
		// 		.or(RobotStates.actionDeAlgaefyL3)
		// 		.whileTrue(
		// 				Commands.sequence(
		// 						algaePivot.waitForIntakeCommand(PivotStates.DEALGAEFY),
		// 						new WaitCommand(pivotDeAlgaefyDelay),
		// 						algaePivot.retractCommand()));

		// Climber bindings
	}

	/** rebinds actions to match one driver controls */
	public void bind1Driver() {
		Swerve swerve = RobotContainer.getSwerve();

		Operator operator = RobotContainer.getOperator();

		// Default commands
		swerve.setDefaultCommand(swerve.teleopSwerveCommand());
	

		// Intake based on last auto align
		// RobotStates.actionIntakeCoral =
		// 		operator.leftDpad.and(RobotStates.hasCoral.negate()).or(RobotStates.isHPAligned);
		// RobotStates.actionIntakeCoralWhileTrue = RobotStates.actionHPAlign;
		// RobotStates.actionGroundAlgae = operator.leftBumper.and(operator.rightBumper.negate());
		// RobotStates.actionReverseCoral = operator.leftDpad.and(RobotStates.hasCoral);

		// Score chaining
		RobotStates.actionScoreAlgae = operator.rightBumper.and(operator.leftBumper.negate());
		RobotStates.actionElevatorL1 = operator.A;
		RobotStates.actionElevatorL2 = operator.X;
		RobotStates.actionElevatorL3 = operator.Y;
		RobotStates.actionElevatorL4 = operator.B;

		// Climber
		RobotStates.actionClimberInSetpoint = operator.upDpad;
		RobotStates.actionClimberOutSetpoint = operator.downDpad;
		RobotStates.actionResetClimber = operator.rightDpad;

		RobotStates.actionResetGyro = operator.start;

		bind2Driver();
	}
}
