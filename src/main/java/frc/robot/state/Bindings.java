package frc.robot.state;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.Indexer;
import frc.robot.subsystems.EndEffector.Indexer.IndexerStates;
import frc.robot.subsystems.EndEffector.Intake;
import frc.robot.subsystems.EndEffector.Intake.IntakeStates;
import frc.robot.subsystems.EndEffector.Shooter;
import frc.robot.subsystems.EndEffector.Shooter.ShooterStates;
import frc.robot.subsystems.EndEffector.Turret;
import frc.robot.subsystems.EndEffector.Turret.TurretStates;
import frc.robot.subsystems.Endgame.Climber;
import frc.robot.subsystems.Endgame.Climber.ClimberState;

public class Bindings {

	/** binds commands to all actions */
	public void bind2Driver() {
		Swerve swerve = RobotContainer.getSwerve();
		Shooter m_Shooter = RobotContainer.getShooter();
		Indexer m_Indexer = RobotContainer.getIndexer();
		Intake m_Intake = RobotContainer.getIntake();
		Turret m_Turret = RobotContainer.getTurret();
		Climber m_Climber = RobotContainer.getClimber();

		// Swerve bindings
		swerve.setDefaultCommand(swerve.teleopSwerveCommand());
		RobotStates.actionResetGyro.whileTrue(swerve.resetHeadingCommand());
		RobotStates.actionXStance.whileTrue(swerve.xStanceCommand());
		RobotStates.actionRobotRelative.onChange(
				new InstantCommand(() -> swerve.toggleFieldRelative()));
		RobotStates.trenchAlignDrive.whileTrue(swerve.trenchAlignTeleopSwerveCommand());

		RobotStates.positionalRotationDrive.whileTrue(swerve.positionRotationTeleopSwerveCommand());

		RobotStates.actionShoot.whileTrue(m_Shooter.setStateCommand(ShooterStates.SHOOTING));

		RobotStates.teleop.onTrue(
				new ParallelCommandGroup(
						m_Indexer.setStateCommandPersistent(IndexerStates.DEFAULT),
						m_Shooter.setStateCommandPersistent(ShooterStates.DEFAULT),
						m_Turret.setStateCommandPersistent(TurretStates.IDLE)));

		// shoot
		RobotStates.actionIndexerShoot
				.and(RobotStates.turretIsInDeadzone.negate())
				.whileTrue(m_Indexer.setStateCommand(IndexerStates.SHOOTING));

		RobotStates.actionIndexerReverse.whileTrue(m_Indexer.setStateCommand(IndexerStates.REVERSING));

		// intake
		RobotStates.actionIntakeDown.whileTrue(m_Intake.setStateCommand(IntakeStates.INTAKING));

		// aim turret (hold)
		RobotStates.actionAimTurretHold.whileTrue(m_Turret.setStateCommand(TurretStates.SCORING));

		// aim turret (toggle)
		RobotStates.actionAimTurretToggle.toggleOnTrue(m_Turret.setStateCommand(TurretStates.SCORING));

		// climber
		RobotStates.actionClimberUp.whileTrue(m_Climber.setStateCommand(ClimberState.RAISING));
		RobotStates.actionClimberDown.whileTrue(m_Climber.setStateCommand(ClimberState.LOWERING));
	}

	/** rebinds actions to match one driver controls */
	public void bind1Driver() {
		bind2Driver();
	}
}
