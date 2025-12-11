package frc.robot.state;

import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class RobotStates {

	public static final Driver m_Driver = RobotContainer.getDriver();
	public static final Operator m_Operator = RobotContainer.getOperator();

	public static final Trigger sim = new Trigger(Robot::isSimulation);
	public static final Trigger teleop = RobotModeTriggers.teleop();
	public static final Trigger auto = RobotModeTriggers.autonomous();
	public static final Trigger disabled = RobotModeTriggers.disabled();

	public static boolean autoL4 = false;
	public static boolean autoScore = false;
	public static boolean autoIntake = false;

	// Swerve actions
	// public static final Trigger actionLeftAlign =
	// m_Driver.getLeftButton(autoAlignButton).and(teleop);
	// public static final Trigger actionRightAlign =
	// 		m_Driver.getRightButton(autoAlignButton).and(teleop);
	// public static final Trigger actionAlgaeAlign =
	// 		m_Driver.getLeftButton(algaeAlignButton).and(teleop);
	// public static final Trigger actionRotateSimilarFace =
	// 		m_Driver.getRightButton(rotateToSimilarFaceButton).and(teleop);
	// public static final Trigger actionHPRotate =
	// m_Driver.getRightButton(hpRotateButton).and(teleop);
	// public static final Trigger actionHPAlign = m_Driver.getLeftButton(hpRotateButton).and(teleop);
	// public static final Trigger actionRobotRelative =
	// 		m_Driver.getRightButton(robotRelativeButton).and(teleop);
	// public static final Trigger actionXStance = m_Driver.getRightButton(xstanceButton).and(teleop);
	// public static final Trigger actionZeroGyro =
	// 		m_Driver.getLeftButton(resetHeadingButton).and(teleop);
	// public static final Trigger actionResetBranchCam =
	// 		m_Driver.getLeftButton(resetBranchCamButton).and(teleop);
}
