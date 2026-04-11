package frc.robot.state;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.Hopper;
import frc.robot.subsystems.EndEffector.Hopper.HopperStates;
import frc.robot.subsystems.EndEffector.Indexer;
import frc.robot.subsystems.EndEffector.Indexer.IndexerStates;
import frc.robot.subsystems.EndEffector.Intake;
import frc.robot.subsystems.EndEffector.Intake.IntakeStates;
import frc.robot.subsystems.EndEffector.Shooter;
import frc.robot.subsystems.EndEffector.Shooter.ShooterStates;
import frc.robot.subsystems.EndEffector.Turret;
import frc.robot.subsystems.EndEffector.Turret.TurretStates;

public class Bindings {

	/** binds commands to all actions */
	public void bind2Driver() {
		Swerve swerve = RobotContainer.getSwerve();
		Shooter m_Shooter = RobotContainer.getShooter();
		Indexer m_Indexer = RobotContainer.getIndexer();
		Hopper m_Hopper = RobotContainer.getHopper();
		Intake m_Intake = RobotContainer.getIntake();
		Turret m_Turret = RobotContainer.getTurret();
		// Climber m_Climber = RobotContainer.getClimber();

		// Swerve bindings
		swerve.setDefaultCommand(swerve.teleopSwerveCommand());
		RobotStates.actionResetGyro.whileTrue(swerve.resetHeadingCommand());
		RobotStates.actionXStance.whileTrue(swerve.xStanceCommand());
		RobotStates.actionRobotRelative.onChange(
				new InstantCommand(() -> swerve.toggleFieldRelative()));
		RobotStates.actionSpeedClamp.onChange(new InstantCommand(() -> swerve.toggleSpeedClamp()));
		RobotStates.trenchAlignDrive.whileTrue(swerve.trenchAlignTeleopSwerveCommand());

		RobotStates.positionalRotationDrive.whileTrue(swerve.positionRotationTeleopSwerveCommand());

		RobotStates.actionShoot.whileTrue(m_Shooter.setStateCommand(ShooterStates.SHOOTING));

		RobotStates.actionDecrementShooter.onTrue(
				new InstantCommand(() -> m_Shooter.decrementAdjustment()));
		RobotStates.actionIncrementShooter.onTrue(
				new InstantCommand(() -> m_Shooter.incrementAdjustment()));

		// shoot
		RobotStates.actionIndexerShoot
				.and(RobotStates.turretIsInDeadzone.negate())
				.and(RobotStates.actionShoot)
				.whileTrue(
						new ParallelCommandGroup(
								m_Indexer.setStateCommand(IndexerStates.SHOOTING),
								m_Hopper.setStateCommand(HopperStates.SHOOTING)));

		RobotStates.actionIndexerReverse.whileTrue(
				new ParallelCommandGroup(
						m_Indexer.setStateCommand(IndexerStates.REVERSING),
						m_Hopper.setStateCommand(HopperStates.REVERSING)));

		// intake
		RobotStates.actionIntakeDown.whileTrue(m_Intake.setStateCommand(IntakeStates.INTAKING));
		RobotStates.actionStowIntake.whileTrue(m_Intake.setStateCommand(IntakeStates.STOWED));
		RobotStates.actionJiggleIntake.whileTrue(m_Intake.jiggleCommand());
		RobotStates.actionReverseIntake.whileTrue(m_Intake.setStateCommand(IntakeStates.REVERSING));

		// aim turret (hold)
		RobotStates.actionAimTurretHold.whileTrue(m_Turret.setStateCommand(TurretStates.SCORING));

		// aim turret (toggle)
		RobotStates.actionAimTurretToggle.toggleOnTrue(m_Turret.setStateCommand(TurretStates.SCORING));

		RobotStates.actionResetTurret.onTrue(new InstantCommand(() -> m_Turret.resetMotorPosition()));

		// climber
		// RobotStates.actionClimberUp.whileTrue(m_Climber.setStateCommand(ClimberState.RAISING));
		// RobotStates.actionClimberDown.whileTrue(m_Climber.setStateCommand(ClimberState.LOWERING));
	}

	/** rebinds actions to match one driver controls */
	public void bind1Driver() {
		bind2Driver();
	}
}
