package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

/*
 * Each command used in auto selector needs to be a separate object
 * This class returns a new instance of a command class based on action number specified
 */

public class ActionFactory {
	private Swerve m_Swerve;
	private Shooter m_Shooter;
	private Intake m_Intake;
	private Indexer m_Indexer;
	private Turret m_Turret;

	public ActionFactory() {
		m_Swerve = RobotContainer.getSwerve();
		m_Shooter = RobotContainer.getShooter();
		m_Intake = RobotContainer.getIntake();
		m_Indexer = RobotContainer.getIndexer();
		m_Turret = RobotContainer.getTurret();
	}

	/** returns command associated with action number */
	public Command getCommand(int action) {
		switch (action) {
			case 1:
				return new ParallelCommandGroup(
						m_Shooter.setStateCommandPersistent(ShooterStates.SHOOTING),
						m_Turret.setStateCommandPersistent(TurretStates.SCORING));
			case 2:
				return new ParallelCommandGroup(
						m_Shooter.setStateCommandPersistent(ShooterStates.DEFAULT),
						m_Turret.setStateCommandPersistent(TurretStates.IDLE));
			case 3:
				return m_Indexer.setStateCommandPersistent(IndexerStates.SHOOTING);
			case 4:
				return m_Indexer.setStateCommandPersistent(IndexerStates.DEFAULT);

			case 5:
				return m_Intake.setStateCommandPersistent(IntakeStates.INTAKING);
			case 6:
				return m_Intake.setStateCommandPersistent(IntakeStates.DEFAULT);
			case 7:
				return new ParallelRaceGroup(m_Shooter.shootXBallsCommand(8), new WaitCommand(3));
			case 8:
				return new SequentialCommandGroup(
								m_Intake.setStateCommandPersistent(IntakeStates.DEFAULT),
								new WaitCommand(1),
								m_Intake.setStateCommandPersistent(IntakeStates.INTAKING),
								new WaitCommand(1))
						.repeatedly()
						.finallyDo(() -> m_Intake.setStateCommandPersistent(IntakeStates.DEFAULT));
		}
		return null;
	}

	/** returns name of command - for display only */
	public String getName(int action) {
		switch (action) {
			case 1:
				return "Start Shooter/Turret";
			case 2:
				return "Stop Shooter/Turret";
			case 3:
				return "Start Indexer";
			case 4:
				return "Stop Indexer";
			case 5:
				return "Start Intaking";
			case 6:
				return "Stop Intaking";
			case 7:
				return "Shoot Preload";
			case 8:
				return "Intake Agitate";
		}
		return null;
	}
}
