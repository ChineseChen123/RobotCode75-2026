package frc.robot.state;

import static frc.robot.Constants.IOConstants.oneDriver;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Swerve;

public class Bindings {

	/** binds commands to all actions */
	public void bind2Driver() {
		Swerve swerve = RobotContainer.getSwerve();

		// Swerve bindings
		swerve.setDefaultCommand(swerve.teleopSwerveCommand());
		RobotStates.actionResetGyro.whileTrue(swerve.resetHeadingCommand());
		RobotStates.actionXStance.whileTrue(swerve.xStanceCommand());
		RobotStates.actionRobotRelative.onChange(
				new InstantCommand(() -> swerve.toggleFieldRelative()));
		RobotStates.trenchAlignDrive.whileTrue(swerve.trenchAlignTeleopSwerveCommand());
	}

	/** rebinds actions to match one driver controls */
	public void bind1Driver() {

		bind2Driver();
	}
}
