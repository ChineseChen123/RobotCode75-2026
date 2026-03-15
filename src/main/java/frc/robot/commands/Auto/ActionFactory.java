package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
						new InstantCommand(
								() -> {
									m_Shooter.setState(ShooterStates.SHOOTING);
								},
								m_Shooter),
						new InstantCommand(
								() -> {
									m_Turret.setState(TurretStates.SCORING);
								},
								m_Turret),
						new InstantCommand(
								() -> {
									m_Indexer.setState(IndexerStates.SHOOTING);
								},
								m_Indexer));
			case 2:
				return new ParallelCommandGroup(
						new InstantCommand(
								() -> {
									m_Shooter.setState(ShooterStates.DEFAULT);
								},
								m_Shooter),
						new InstantCommand(
								() -> {
									m_Indexer.setState(IndexerStates.DEFAULT);
								},
								m_Indexer),
						new InstantCommand(
								() -> {
									m_Turret.setState(TurretStates.IDLE);
								},
								m_Turret));

			case 3:
				return new InstantCommand(
						() -> {
							m_Intake.setState(IntakeStates.INTAKING);
						},
						m_Intake);
			case 4:
				return new InstantCommand(
						() -> {
							m_Intake.setState(IntakeStates.STOWED);
						},
						m_Intake);
			case 5:
				return new ParallelRaceGroup(m_Shooter.shootXBallsCommand(8),
						new WaitCommand(3));
		}
		return null;
	}

	/** returns name of command - for display only */
	public String getName(int action) {
		switch (action) {
			case 1:
				return "Start Shooting";
			case 2:
				return "Stop Shooting";
			case 3:
				return "Start Intaking";
			case 4:
				return "Stop Intaking";
		}
		return null;
	}
}
