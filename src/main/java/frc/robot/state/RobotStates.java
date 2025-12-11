package frc.robot.state;

import static frc.robot.Constants.IOConstants.*;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.Offset;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain.controllers.AutoAlign;
import frc.robot.subsystems.Drivetrain.controllers.AutoAlign.HPAlign;
import frc.robot.subsystems.Drivetrain.controllers.AutoAlign.ReefAlign;
import frc.robot.subsystems.EndEffector.AlgaeIntake.AlgaeStates;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.AlgaePivot.PivotStates;
import frc.robot.subsystems.EndEffector.CoralIntake.CoralStates;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;

public class RobotStates {

	public static final Driver m_Driver = RobotContainer.getDriver();
	public static final Operator m_Operator = RobotContainer.getOperator();

	public static final Elevator m_Elevator = RobotContainer.getElevator();
	public static final AlgaePivot m_Pivot = RobotContainer.getAlgaePivot();
	public static final Climber m_Climber = RobotContainer.getClimber();

	/** Game time triggers */
	public static final Trigger sim = new Trigger(Robot::isSimulation);

	public static final Trigger teleop = RobotModeTriggers.teleop();
	public static final Trigger auto = RobotModeTriggers.autonomous();
	public static final Trigger disabled = RobotModeTriggers.disabled();
	public static final Trigger endgame = null; // TODO implement

	public static boolean autoL4 = false;
	public static boolean autoScore = false;
	public static boolean autoIntake = false;

	// ── States ──────────────────────────────────────────────────────────────────

	/** Elevator states */
	public static final Trigger elevatorAtHome =
			new Trigger(() -> m_Elevator.isAtPosition(ElevatorPositions.HOME, false));

	public static final Trigger elevatorAtL1 =
			new Trigger(() -> m_Elevator.isAtPosition(ElevatorPositions.L1, false));
	public static final Trigger elevatorAtL2 =
			new Trigger(() -> m_Elevator.isAtPosition(ElevatorPositions.L2, false));
	public static final Trigger elevatorAtL2Algae =
			new Trigger(() -> m_Elevator.isAtPosition(ElevatorPositions.L2, true));
	public static final Trigger elevatorAtL3 =
			new Trigger(() -> m_Elevator.isAtPosition(ElevatorPositions.L3, false));
	public static final Trigger elevatorAtL3Algae =
			new Trigger(() -> m_Elevator.isAtPosition(ElevatorPositions.L3, true));
	public static final Trigger elevatorAtL4 =
			new Trigger(() -> m_Elevator.isAtPosition(ElevatorPositions.L4, false));
	public static final Trigger elevatorBelowL3 =
			new Trigger(() -> m_Elevator.isBelowPosition(ElevatorPositions.L3, false)); // Auto only
	public static final Trigger elevatorAtNet =
			new Trigger(() -> m_Elevator.isAtPosition(ElevatorPositions.NET, false));
	public static final Trigger elevatorAtProcessor =
			new Trigger(() -> m_Elevator.isAtPosition(ElevatorPositions.PROCESSOR, false));

	/** Coral Intake states */
	public static final Trigger hasCoral = new Trigger(RobotContainer.getCoralIntake()::hasCoral);

	public static final Trigger isCoralPositioned =
			new Trigger(RobotContainer.getCoralIntake()::atPosition);
	public static final Trigger isCoralIntaking =
			new Trigger(() -> RobotContainer.getCoralIntake().getState() == CoralStates.INTAKING);

	/** Algae Intake states */
	public static final Trigger hasAlgae = new Trigger(RobotContainer.getAlgaeIntake()::hasAlgae);

	public static final Trigger isAlgaeIntaking =
			new Trigger(() -> RobotContainer.getAlgaeIntake().getState() == AlgaeStates.INTAKING);

	/** Alge Pivot states */
	public static final Trigger pivotAtHome =
			new Trigger(() -> m_Pivot.isAtPosition(PivotStates.RETRACTED));

	public static final Trigger pivotAtDealgaefy =
			new Trigger(() -> m_Pivot.isAtPosition(PivotStates.DEALGAEFY));
	public static final Trigger pivotAtGround =
			new Trigger(() -> m_Pivot.isAtPosition(PivotStates.GROUNDINTAKE));
	public static final Trigger pivotAtProcessor =
			new Trigger(() -> m_Pivot.isAtPosition(PivotStates.PROCESSOR));

	/** Climber states */
	public static final Trigger isClimberReset =
			new Trigger(() -> (m_Climber.isReset() && m_Climber.getPositionRotations() > 5));

	public static final Trigger climberLimit = new Trigger(m_Climber::getLimitSwitch);

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
	public static final Trigger isAlgaeL3 =
			new Trigger(() -> (RobotContainer.getSwerve().algaeLevel() == "L3"));
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
	public static Trigger actionLeftAlign = m_Driver.getLeftButton(autoAlignButton).and(teleop);

	public static Trigger actionRightAlign = m_Driver.getRightButton(autoAlignButton).and(teleop);
	public static Trigger actionAlgaeAlign = m_Driver.getLeftButton(algaeAlignButton).and(teleop);
	public static Trigger actionRotateSimilarFace =
			m_Driver.getRightButton(rotateToSimilarFaceButton).and(teleop);
	public static Trigger actionHPRotate = m_Driver.getRightButton(hpRotateButton).and(teleop);
	public static Trigger actionHPAlign = m_Driver.getLeftButton(hpRotateButton).and(teleop);
	public static Trigger actionRobotRelative =
			m_Driver.getRightButton(robotRelativeButton).and(teleop);
	public static Trigger actionXStance = m_Driver.getRightButton(xstanceButton).and(teleop);
	public static Trigger actionResetGyro = m_Driver.getLeftButton(resetHeadingButton).and(teleop);
	public static Trigger actionResetBranchCam =
			m_Driver.getLeftButton(resetBranchCamButton).and(teleop);
}
