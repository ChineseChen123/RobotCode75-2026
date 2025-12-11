package frc.robot.state;

import static frc.robot.Constants.IOConstants.*;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.Offset;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.controllers.AutoAlign;
import frc.robot.subsystems.Drivetrain.controllers.AutoAlign.HPAlign;
import frc.robot.subsystems.Drivetrain.controllers.AutoAlign.ReefAlign;

public class RobotStates {

	public static final Driver m_Driver = RobotContainer.getDriver();
	public static final Operator m_Operator = RobotContainer.getOperator();

	/** Game time triggers */
	public static final Trigger sim = new Trigger(Robot::isSimulation);

	public static final Trigger teleop = RobotModeTriggers.teleop();
	public static final Trigger auto = RobotModeTriggers.autonomous();
	public static final Trigger disabled = RobotModeTriggers.disabled();
	public static final Trigger endgame = null;

	public static boolean autoL4 = false;
	public static boolean autoScore = false;
	public static boolean autoIntake = false;

	/** Auto align states */
	public static final Trigger isLeftAligned =
			new Trigger(
					() ->
							(AutoAlign.isInitialized()
									&& ReefAlign.isFinished()
									&& ReefAlign.getOffset() == Offset.LEFT));

	public static final Trigger isRightAligned =
			new Trigger(
					() ->
							(AutoAlign.isInitialized()
									&& ReefAlign.isFinished()
									&& ReefAlign.getOffset() == Offset.RIGHT));
	public static final Trigger isAlgaeAligned =
			new Trigger(
					() ->
							(AutoAlign.isInitialized()
									&& ReefAlign.isFinished()
									&& ReefAlign.getOffset() == Offset.MID));
	public static final Trigger isHPAligned =
			new Trigger(() -> (AutoAlign.isInitialized() && HPAlign.isFinished()));
	public static final Trigger isLastTargetReef =
			new Trigger(() -> FieldPose.fieldElementIsReef(AutoAlign.lastAutoAlign()));
	public static final Trigger isLastTargetHP =
			new Trigger(() -> FieldPose.fieldElementIsHPStation(AutoAlign.lastAutoAlign()));

	// ── Actions ──────────────────────────────────────────────────────────────────

	/** Elevator actions */
	public static Trigger actionElevatorL1 = m_Operator.A.and(teleop);

	public static Trigger actionElevatorL2 =
			m_Operator.X.and(m_Operator.leftTriggerGreater(0.15).negate()).and(teleop);
	public static Trigger actionElevatorL3 =
			m_Operator.Y.and(m_Operator.leftTriggerGreater(0.15).negate()).and(teleop);
	public static Trigger actionElevatorL4 =
			m_Operator.B.and(m_Operator.leftTriggerGreater(0.15).negate()).and(teleop).or(() -> autoL4);

	/** Coral actions */
	public static Trigger actionScoreCoral =
			m_Operator.rightTriggerGreater(0.15).and(teleop).or(() -> autoScore);

	public static Trigger actionIntakeCoral = m_Operator.upDpad.and(teleop).or(() -> autoIntake);
	public static Trigger actionIntakeCoralWhileTrue = null;
	public static Trigger actionReverseCoral = m_Operator.leftDpad.and(teleop);

	/** Algae actions */
	public static Trigger actionScoreAlgae = m_Operator.rightBumper.and(teleop);

	public static Trigger actionGroundAlgae = m_Operator.leftBumper.and(teleop);
	public static Trigger actionProcessorPivot = m_Operator.rightDpad.and(teleop);
	public static Trigger actionDeAlgaefyL2 =
			m_Operator.X.and(m_Operator.leftTriggerGreater(0.15)).and(teleop);
	public static Trigger actionDeAlgaefyL3 =
			m_Operator.Y.and(m_Operator.leftTriggerGreater(0.15)).and(teleop);
	public static Trigger actionElevatorNet =
			m_Operator.B.and(m_Operator.leftTriggerGreater(0.15)).and(teleop);

	/** Climber actions */
	public static Trigger actionClimberIn =
			new Trigger(() -> m_Operator.leftStickY().getAsDouble() < -0.15);

	public static Trigger actionClimberOut =
			new Trigger(() -> m_Operator.leftStickY().getAsDouble() > 0.15);
	public static Trigger actionClimberInSetpoint =
			new Trigger(() -> m_Operator.rightStickY().getAsDouble() < -0.15);
	public static Trigger actionClimberOutSetpoint =
			new Trigger(() -> m_Operator.rightStickY().getAsDouble() > 0.15);
	public static Trigger actionResetClimber = m_Operator.downDpad;

	/** Swerve actions */
	public static Trigger actionRobotRelative =
			m_Driver.getRightButton(robotRelativeButton).and(teleop);
	public static Trigger actionXStance = m_Driver.getRightButton(xstanceButton).and(teleop);
	public static Trigger actionResetGyro = m_Driver.getLeftButton(resetHeadingButton).and(teleop);
}
