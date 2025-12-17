package frc.lib.dashboard;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Swerve;

/*
 * Each command used in auto selector needs to be a separate object
 * This class returns a new instance of a command class based on action number specified
 */

public class ActionFactory {
	private Swerve m_Swerve;

	public ActionFactory() {
		m_Swerve = RobotContainer.getSwerve();
	}

	/** returns command associated with action number */
	public Command getCommand(int action) {
		switch (action) {
		}
		return null;
	}

	/** returns name of command - for display only */
	public String getName(int action) {
		switch (action) {
		}
		return null;
	}
}
