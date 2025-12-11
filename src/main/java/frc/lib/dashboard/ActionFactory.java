package frc.lib.dashboard;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
// import frc.robot.commands.Autonomous.AutoIntakeCoral;
// import frc.robot.commands.Autonomous.AutoScoreL1;
// import frc.robot.commands.Autonomous.AutoScoreL4;
// import frc.robot.commands.Autonomous.AutoScoreProcessor;
// import frc.robot.commands.Autonomous.ChoppedPreraiseElevator;
// import frc.robot.commands.Drivetrain.YoloBranchAlign;
// import frc.robot.state.RobotStates;
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
		// 1 - L1, 2 - Dealgaefy, 3 - L4, 4 - Processor, 5 - Intake
		switch (action) {
			case 1:
				return null;
				// case 2:
				// 	return new AutoDealgaefy(
				// 		m_Swerve, m_Elevator, m_AlgaeIntake, m_AlgaePivot, m_ChezyController);
			case 3:
				return null;
			case 4:
				return null;
			case 5:
				return null;
			case 6:
				return null;
			case 7:
				return null;
			case 8:
				return null;
			case 9:
				return null;
		}
		return null;
	}

	/** returns name of command - for display only */
	public String getName(int action) {
		switch (action) {
			case 1:
				return "L1";
				// case 2:
				//   return "Dealgaefy";
			case 3:
				return "L4";
			case 4:
				return "Processor";
			case 5:
				return "Intake";
			case 6:
				return "YOLO";
			case 7:
				return "Preraise Elevator";
			case 8:
				return "YOLO+L4";
			case 9:
				return "L4 Fallback";
		}
		return null;
	}
}
