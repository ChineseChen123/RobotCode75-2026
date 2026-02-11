package frc.robot.state;

import static frc.robot.Constants.IOConstants.*;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class RobotStates {

	public static final Driver m_Driver = RobotContainer.getDriver();
	public static final Operator m_Operator = RobotContainer.getOperator();

	/** Game time triggers */
	public static final Trigger sim = new Trigger(Robot::isSimulation);

	public static final Trigger teleop = RobotModeTriggers.teleop();
	public static final Trigger auto = RobotModeTriggers.autonomous();
	public static final Trigger disabled = RobotModeTriggers.disabled();
	public static final Trigger endgame = null;

	// ── States ──────────────────────────────────────────────────────────────────

	/** Intake & Pivot states */

	/** */

	// ── Actions ──────────────────────────────────────────────────────────────────

	// ** Intake Actions */
	public static Trigger actionIntake = m_Operator.A.and(teleop);

	public static Trigger actionIntakeReverse = m_Operator.B.and(teleop);

	/** Swerve actions */
	public static Trigger actionRobotRelative =
			m_Driver.getRightButton(robotRelativeButton).and(teleop);

	public static Trigger actionXStance = m_Driver.getRightButton(xstanceButton).and(teleop);
	public static Trigger actionResetGyro = m_Driver.getLeftButton(resetHeadingButton).and(teleop);
}
