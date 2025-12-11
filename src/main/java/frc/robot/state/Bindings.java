package frc.robot.state;

import static frc.robot.Constants.ClimberConstants.climbExtendPosition;
import static frc.robot.Constants.ClimberConstants.climbPosition;
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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaeIntake.AlgaeStates;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.AlgaePivot.PivotStates;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.CoralIntake.CoralStates;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;

public class Bindings {

	/** binds commands to all actions */
	public void bind2Driver() {
		Swerve swerve = RobotContainer.getSwerve();
		Elevator elevator = RobotContainer.getElevator();
		CoralIntake coralIntake = RobotContainer.getCoralIntake();
		AlgaeIntake algaeIntake = RobotContainer.getAlgaeIntake();
		AlgaePivot algaePivot = RobotContainer.getAlgaePivot();
		Climber climber = RobotContainer.getClimber();

		// Swerve bindings
		swerve.setDefaultCommand(swerve.teleopSwerveCommand());
		RobotStates.actionResetGyro.whileTrue(swerve.resetHeadingCommand());
		if (!oneDriver) {
			RobotStates.actionXStance.whileTrue(swerve.xStanceCommand());
		}
		RobotStates.actionLeftAlign.whileTrue(swerve.reefAlignCommand(Offset.LEFT));
		RobotStates.actionRightAlign.whileTrue(swerve.reefAlignCommand(Offset.RIGHT));
		RobotStates.actionAlgaeAlign.whileTrue(swerve.reefAlignCommand(Offset.MID));
		// RobotStates.actionRotateSimilarFace.whileTrue(swerve.similarFaceRotateCommand());
		if (!oneDriver) {
			RobotStates.actionHPRotate.whileTrue(swerve.hpRotateCommand());
		}
		RobotStates.actionHPAlign.whileTrue(swerve.hpAlignCommand(Offset.MID));
		RobotStates.actionRobotRelative.onChange(
				new InstantCommand(() -> swerve.toggleFieldRelative()));
		RobotStates.actionResetBranchCam.onTrue(
				new InstantCommand(() -> RobotContainer.getBranchCamera().reloadPipeline())
						.ignoringDisable(true));

		// Elevator bindings
		elevator.setDefaultCommand(elevator.positionCommand(ElevatorPositions.HOME, false));
		// Solo raise elevator commands
		RobotStates.actionElevatorL1.whileTrue(elevator.positionCommand(ElevatorPositions.L1, false));
		RobotStates.actionElevatorL2.whileTrue(elevator.positionCommand(ElevatorPositions.L2, false));
		RobotStates.actionElevatorL3.whileTrue(elevator.positionCommand(ElevatorPositions.L3, false));
		RobotStates.actionElevatorL4.whileTrue(elevator.positionCommand(ElevatorPositions.L4, false));
		if (!oneDriver) {
			RobotStates.actionElevatorNet.whileTrue(
					elevator.positionCommand(ElevatorPositions.NET, false));
		}
		// DeAlgaefy - lower only after pivot retracts
		RobotStates.actionDeAlgaefyL2.whileTrue(
				elevator
						.positionCommand(ElevatorPositions.L2, true)
						.until(RobotStates.pivotAtHome.and(RobotStates.hasAlgae)));
		RobotStates.actionDeAlgaefyL3.whileTrue(
				elevator
						.positionCommand(ElevatorPositions.L3, true)
						.until(RobotStates.pivotAtHome.and(RobotStates.hasAlgae)));
		// Brief raise so algae doesn't scrape the carpet after ground intaking
		RobotStates.actionGroundAlgae
				.and(RobotStates.hasAlgae)
				.onTrue(elevator.positionCommandUntilDone(ElevatorPositions.L1, false));

		// Coral Intake bindings
		RobotStates.actionIntakeCoral
				.and(RobotStates.hasCoral.negate())
				.toggleOnTrue(coralIntake.intakeCommand());
		// Score speed based on elevator position
		RobotStates.actionScoreCoral
				.and(RobotStates.hasCoral)
				.whileTrue(
						coralIntake.setStateCommand(
								RobotStates.elevatorAtL1.getAsBoolean()
										? CoralStates.SCOREL1
										: RobotStates.elevatorAtL4.getAsBoolean()
												? CoralStates.SCOREL4
												: CoralStates.SCOREL23));
		RobotStates.actionReverseCoral.whileTrue(coralIntake.setStateCommand(CoralStates.REVERSING));
		// Reset intaking timer when not intaking
		RobotStates.isCoralIntaking.whileFalse(coralIntake.updateTimerCommand());

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
		climber.setDefaultCommand(
				new InstantCommand(() -> climber.runCurrent(0), climber).repeatedly());
		// Run by current based on stick value
		RobotStates.actionResetClimber.onTrue(
				Commands.sequence(
						new InstantCommand(() -> climber.runCurrent(60), climber, elevator)
								.repeatedly()
								.until(RobotStates.climberLimit),
						new InstantCommand(() -> climber.runCurrent(-60), climber, elevator)
								.repeatedly()
								.until(RobotStates.isClimberReset)));
		if (!oneDriver) {
			RobotStates.actionClimberIn
					.or(RobotStates.actionClimberOut)
					.whileTrue(
							new InstantCommand(
											() ->
													climber.runCurrent(
															-RobotContainer.getOperator().leftStickY().getAsDouble() * 75),
											climber)
									.repeatedly());
		}
		// Run to setpoint until done
		RobotStates.actionClimberInSetpoint.whileTrue(
				new InstantCommand(() -> climber.setPositionRequest(climbPosition), climber).repeatedly());
		RobotStates.actionClimberOutSetpoint.whileTrue(
				new InstantCommand(() -> climber.setPositionRequest(climbExtendPosition), climber)
						.repeatedly());
	}

	/** rebinds actions to match one driver controls */
	public void bind1Driver() {
		Swerve swerve = RobotContainer.getSwerve();
		Elevator elevator = RobotContainer.getElevator();
		CoralIntake coralIntake = RobotContainer.getCoralIntake();
		AlgaeIntake algaeIntake = RobotContainer.getAlgaeIntake();
		AlgaePivot algaePivot = RobotContainer.getAlgaePivot();
		Climber climber = RobotContainer.getClimber();
		Operator operator = RobotContainer.getOperator();

		// Default commands
		swerve.setDefaultCommand(swerve.teleopSwerveCommand());
		elevator.setDefaultCommand(elevator.positionCommand(ElevatorPositions.HOME, false));
		algaePivot.setDefaultCommand(algaePivot.setStateCommand(PivotStates.RETRACTED));
		climber.setDefaultCommand(
				new InstantCommand(() -> climber.runCurrent(0), climber).repeatedly());

		// LT/RT auto align based on coral state
		RobotStates.actionLeftAlign =
				operator.leftTriggerGreater(0.15).and(operator.rightTriggerGreater(0.15).negate());
		RobotStates.actionRightAlign =
				operator.rightTriggerGreater(0.15).and(operator.leftTriggerGreater(0.15).negate());
		RobotStates.actionHPAlign = operator.leftBumper.and(operator.rightBumper);

		// Dealgaefy chain
		RobotStates.actionAlgaeAlign =
				operator.leftTriggerGreater(0.15).and(operator.rightTriggerGreater(0.15));
		RobotStates.actionDeAlgaefyL2 =
				RobotStates.actionAlgaeAlign
						.and(RobotStates.isAlgaeAligned)
						.and(RobotStates.isAlgaeL3.negate());
		RobotStates.actionDeAlgaefyL3 =
				RobotStates.actionAlgaeAlign.and(RobotStates.isAlgaeAligned).and(RobotStates.isAlgaeL3);
		RobotStates.actionAlgaeAlign
				.and(RobotStates.hasAlgae)
				.whileTrue(
						Commands.race(
								new WaitCommand(0.5),
								new InstantCommand(
												() -> swerve.setRobotRelative(new ChassisSpeeds(-0.5, 0, 0)), swerve)
										.repeatedly()));

		// Intake based on last auto align
		RobotStates.actionIntakeCoral =
				operator.leftDpad.and(RobotStates.hasCoral.negate()).or(RobotStates.isHPAligned);
		RobotStates.actionIntakeCoralWhileTrue = RobotStates.actionHPAlign;
		RobotStates.actionGroundAlgae = operator.leftBumper.and(operator.rightBumper.negate());
		RobotStates.actionReverseCoral = operator.leftDpad.and(RobotStates.hasCoral);

		// Score chaining
		RobotStates.actionScoreAlgae = operator.rightBumper.and(operator.leftBumper.negate());
		RobotStates.actionElevatorL1 = operator.A;
		RobotStates.actionElevatorL2 = operator.X;
		RobotStates.actionElevatorL3 = operator.Y;
		RobotStates.actionElevatorL4 = operator.B;
		RobotStates.actionScoreCoral =
				operator.A.or(operator.X.and(RobotStates.elevatorAtL2))
						.or(operator.Y.and(RobotStates.elevatorAtL3))
						.or(operator.B.and(RobotStates.elevatorAtL4));

		// Climber
		RobotStates.actionClimberInSetpoint = operator.upDpad;
		RobotStates.actionClimberOutSetpoint = operator.downDpad;
		RobotStates.actionResetClimber = operator.rightDpad;

		RobotStates.actionResetGyro = operator.start;
		RobotStates.actionResetBranchCam = operator.back;

		bind2Driver();

		// Intake while HP align
		RobotStates.actionIntakeCoralWhileTrue
				.and(RobotStates.hasCoral.negate())
				.whileTrue(coralIntake.intakeCommand());

		// Split algae scoring by position
		RobotStates.actionScoreAlgae
				.and(() -> FieldPose.fieldElementIsBarge(PeddieBounds.nearestElement(swerve.getPose())))
				.whileTrue(
						Commands.sequence(
								elevator.positionCommandUntilDone(ElevatorPositions.NET, false),
								algaeIntake
										.setStateCommand(AlgaeStates.SCORING)
										.alongWith(
												new InstantCommand(() -> swerve.xStance(), elevator, swerve).repeatedly())
										.until(RobotStates.hasAlgae.negate()),
								new WaitCommand(0.75)
										.alongWith(
												new InstantCommand(() -> swerve.xStance(), elevator, swerve)
														.repeatedly())));
		RobotStates.actionScoreAlgae
				.and(() -> (PeddieBounds.nearestElement(swerve.getPose()) == FieldElement.P))
				.whileTrue(
						Commands.sequence(
								algaePivot.setStateCommand(PivotStates.PROCESSOR),
								new WaitCommand(1),
								algaeIntake
										.setStateCommand(AlgaeStates.SCORING)
										.until(RobotStates.hasAlgae.negate()),
								algaePivot.setStateCommand(PivotStates.PROCESSOR)));
		RobotStates.actionScoreAlgae
				.and(
						() ->
								(!FieldPose.fieldElementIsBarge(PeddieBounds.nearestElement(swerve.getPose()))
										&& PeddieBounds.nearestElement(swerve.getPose()) != FieldElement.P))
				.whileTrue(algaeIntake.setStateCommand(AlgaeStates.SCORING));
	}
}
