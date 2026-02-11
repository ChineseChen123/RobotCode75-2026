package frc.robot.state;

import frc.robot.RobotContainer;
import frc.robot.subsystems.EndEffector.Intake;
import frc.robot.subsystems.EndEffector.Intake.IntakeStates;

public class Bindings {

	/** binds commands to all actions */
	public void bind2Driver() {
		// Swerve swerve = RobotContainer.getSwerve();
		Intake intake = RobotContainer.getIntake();

		// // Swerve bindings
		// swerve.setDefaultCommand(swerve.teleopSwerveCommand());
		// RobotStates.actionResetGyro.whileTrue(swerve.resetHeadingCommand());
		// if (!oneDriver) {
		// 	RobotStates.actionXStance.whileTrue(swerve.xStanceCommand());
		// }
		// RobotStates.actionRobotRelative.onChange(
		// 		new InstantCommand(() -> swerve.toggleFieldRelative()));

		intake.setDefaultCommand(intake.setStateCommand(IntakeStates.DEFAULT));

		RobotStates.actionIntake.whileTrue(intake.setStateCommand(IntakeStates.INTAKING));
		RobotStates.actionIntakeReverse.whileTrue(intake.setStateCommand(IntakeStates.REVERSING));
	}

	/** rebinds actions to match one driver controls */
	public void bind1Driver() {

		bind2Driver();
	}
}
