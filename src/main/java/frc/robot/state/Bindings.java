package frc.robot.state;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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

public class Bindings {

	/** binds commands to all actions */
	public void bind2Driver() {
		Swerve swerve = RobotContainer.getSwerve();
		Shooter m_Shooter = RobotContainer.getShooter();
		Indexer m_Indexer = RobotContainer.getIndexer();
		Intake m_Intake = RobotContainer.getIntake();
		Turret m_Turret = RobotContainer.getTurret();

		// Swerve bindings
		swerve.setDefaultCommand(swerve.teleopSwerveCommand());
		RobotStates.actionResetGyro.whileTrue(swerve.resetHeadingCommand());
		RobotStates.actionXStance.whileTrue(swerve.xStanceCommand());
		RobotStates.actionRobotRelative.onChange(
				new InstantCommand(() -> swerve.toggleFieldRelative()));
		RobotStates.trenchAlignDrive.whileTrue(swerve.trenchAlignTeleopSwerveCommand());

		RobotStates.positionalRotationDrive.whileTrue(swerve.positionRotationTeleopSwerveCommand());

		RobotStates.actionShoot.whileTrue(
				new InstantCommand(
								() -> {
									m_Shooter.setState(ShooterStates.SHOOTING);
								},
								m_Shooter)
						.repeatedly()
						.finallyDo(
								() -> {
									m_Shooter.setState(ShooterStates.DEFAULT);
								}));

		// shoot
		RobotStates.actionIndexerShoot.whileTrue(
				new InstantCommand(
								() -> {
									m_Indexer.setState(
											RobotStates.turretIsInDeadzone.getAsBoolean()
													? IndexerStates.DEFAULT
													: IndexerStates.SHOOTING);
								},
								m_Indexer)
						.repeatedly()
						.finallyDo(
								() -> {
									m_Indexer.setState(IndexerStates.DEFAULT);
								}));

		RobotStates.actionIndexerReverse.whileTrue(
				new InstantCommand(
								() -> {
									m_Indexer.setState(IndexerStates.REVERSING);
								},
								m_Indexer)
						.repeatedly()
						.finallyDo(
								() -> {
									m_Indexer.setState(IndexerStates.DEFAULT);
								}));

		// intake
		RobotStates.actionIntakeDown.whileTrue(
				new InstantCommand(
								() -> {
									m_Intake.setState(IntakeStates.INTAKING);
								},
								m_Intake)
						.repeatedly()
						.finallyDo(
								() -> {
									m_Intake.setState(IntakeStates.DEFAULT);
								}));

		// aim turret (hold)
		RobotStates.actionAimTurretHold.whileTrue(
				new InstantCommand(
								() -> {
									m_Turret.setState(TurretStates.SCORING);
								},
								m_Turret)
						.repeatedly()
						.finallyDo(() -> m_Turret.setState(TurretStates.IDLE)));

		// aim turret (toggle)
		RobotStates.actionAimTurretToggle.whileTrue(
				new InstantCommand(
						() -> {
							m_Turret.setState(
									m_Turret.getTurretState() != TurretStates.SCORING
											? TurretStates.SCORING
											: TurretStates.IDLE);
						},
						m_Turret));
	}

	/** rebinds actions to match one driver controls */
	public void bind1Driver() {

		bind2Driver();
	}
}
