package frc.robot.state;

import static frc.robot.Constants.IOConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.Shooter;
import frc.robot.subsystems.EndEffector.Turret;
import frc.robot.subsystems.EndEffector.Turret.TurretStates;
import java.util.function.Supplier;

public class RobotStates {

	public static final Driver m_Driver = RobotContainer.getDriver();
	public static final Operator m_Operator = RobotContainer.getOperator();

	public static final Swerve m_Swerve = RobotContainer.getSwerve();

	// public static final Intake m_Intake = RobotContainer.getIntake();
	// public static final Indexer m_Indexer = RobotContainer.getIndexer();
	public static final Shooter m_Shooter = RobotContainer.getShooter();
	public static final Turret m_Turret = RobotContainer.getTurret();

	/** Game time triggers */
	public static final Trigger sim = new Trigger(Robot::isSimulation);

	public static final Trigger teleop = RobotModeTriggers.teleop();
	public static final Trigger auto = RobotModeTriggers.autonomous();
	public static final Trigger disabled = RobotModeTriggers.disabled();
	public static final Trigger endgame = null;

	// ── States ───────────────────────────────────────────────────────────────────

	/** Robot info suppliers */
	public static final Supplier<Pose2d> robotPose = m_Swerve::getPose;

	public static final Supplier<Rotation2d> robotHeading = m_Swerve::getHeading;
	public static final Supplier<ChassisSpeeds> fieldRelativeSpeeds =
			m_Swerve::getFieldRelativeChassisSpeeds;

	/** Robot pose info */
	// public static final Trigger isInNeutralZone =
	// 		new Trigger(() -> PeddieBounds.isInNeutralZone(robotPose.get()));

	// public static final Trigger isInOwnZone =
	// 		new Trigger(() -> PeddieBounds.isInOwnZone(robotPose.get()));
	// public static final Trigger isInTrench =
	// 		new Trigger(() -> PeddieBounds.isInTrench(robotPose.get()));

	/** Intake states */
	// public static final Trigger isIntakeDown =
	// 		new Trigger(() -> m_Intake.isAtPosition(IntakeStates.INTAKING));

	// public static final Trigger isIntakeUp =
	// 		new Trigger(() -> m_Intake.isAtPosition(IntakeStates.STOWED));

	/** Indexer states */

	/** Shooter states */
	public static final Trigger shooterAtSpeed = new Trigger(m_Shooter::atTargetVelocity);

	/** Turret states */
	// public static final Trigger turretFullyReset = new Trigger(() -> m_Turret.resetState() == 2);

	public static final Trigger turretAtTarget = new Trigger(m_Turret::atTargetHeading);

	public static final Trigger turretIsStowed = new Trigger(m_Turret::isStowed);
	public static final Trigger turretIsInDeadzone = new Trigger(m_Turret::inDeadzone);
	public static final Trigger turretIsAligning =
			new Trigger(() -> m_Turret.getTurretState() == TurretStates.SCORING);

	// ── Actions ──────────────────────────────────────────────────────────────────

	/** Swerve actions */
	public static Trigger actionRobotRelative =
			m_Driver.getRightButton(robotRelativeButton).and(teleop);

	public static Trigger actionXStance = m_Driver.getRightButton(xstanceButton).and(teleop);
	public static Trigger actionResetGyro = m_Driver.getLeftButton(resetHeadingButton).and(teleop);

	public static Trigger trenchAlignDrive = m_Driver.getLeftButton(trenchDriveButton).and(teleop);
	public static Trigger positionalRotationDrive =
			m_Driver.getRightButton(trenchDriveButton).and(teleop);

	/* Intake actions */

	public static Trigger actionIntakeDown = m_Operator.A.and(teleop);

	/* Indexer actions */

	public static Trigger actionIndexerShoot = m_Operator.rightTriggerGreater(.15).and(teleop);
	public static Trigger actionIndexerReverse = m_Operator.rightBumper.and(teleop);

	/* Shooter actions */

	public static Trigger actionShoot = m_Operator.rightTriggerGreater(.15).and(teleop);

	/* Turret actions */

	public static Trigger actionAimTurretHold = m_Operator.B.and(teleop);
	public static Trigger actionAimTurretToggle = m_Operator.upDpad.and(teleop);
}
