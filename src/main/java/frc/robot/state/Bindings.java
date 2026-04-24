package frc.robot.state;

import static frc.robot.Constants.IOConstants.oneDriver;
import static frc.robot.Constants.IOConstants.operatorDeadband;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
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
import frc.robot.subsystems.Endgame.Climber;

public class Bindings {

	/** binds commands to all actions */
	public void bind2Driver() {
		Swerve swerve = RobotContainer.getSwerve();
		Shooter shooter = RobotContainer.getShooter();
		Indexer indexer = RobotContainer.getIndexer();
		Hopper hopper = RobotContainer.getHopper();
		Intake intake = RobotContainer.getIntake();
		Turret turret = RobotContainer.getTurret();
		// Climber climber = RobotContainer.getClimber();

		// Swerve bindings
		swerve.setDefaultCommand(swerve.teleopSwerveCommand());
		RobotStates.actionResetGyro.whileTrue(swerve.resetHeadingCommand());
		RobotStates.actionXStance.whileTrue(swerve.xStanceCommand());
		RobotStates.actionRobotRelative.onChange(
				new InstantCommand(() -> swerve.toggleFieldRelative()));
		RobotStates.actionSpeedClamp.onChange(new InstantCommand(() -> swerve.toggleSpeedClamp()));
		if (!oneDriver) {
			RobotStates.trenchAlignDrive.whileTrue(swerve.trenchAlignTeleopSwerveCommand());

			RobotStates.positionalRotationDrive.whileTrue(swerve.positionRotationTeleopSwerveCommand());
		}
		RobotStates.actionShoot.whileTrue(shooter.setStateCommand(ShooterStates.SHOOTING));

		RobotStates.actionDecrementShooter.onTrue(
				new InstantCommand(() -> shooter.decrementAdjustment()));
		RobotStates.actionIncrementShooter.onTrue(
				new InstantCommand(() -> shooter.incrementAdjustment()));

		RobotStates.actionReverseShooter.whileTrue(shooter.setStateCommand(ShooterStates.REVERSING));

		// shoot
		RobotStates.actionIndexerShoot
				.and(RobotStates.turretIsInDeadzone.negate())
				.and(RobotStates.actionShoot)
				.and(RobotStates.shooterGoodToShoot)
				.whileTrue(
						new ParallelCommandGroup(
								indexer.setStateCommand(IndexerStates.SHOOTING),
								hopper.setStateCommand(HopperStates.SHOOTING)));

		RobotStates.actionIndexerReverse.whileTrue(
				new ParallelCommandGroup(
						indexer.setStateCommand(IndexerStates.REVERSING),
						hopper.setStateCommand(HopperStates.REVERSING)));

		// intake
		RobotStates.actionIntakeDown.whileTrue(intake.setStateCommand(IntakeStates.INTAKING));
		if (!oneDriver) {
			RobotStates.actionStowIntake.whileTrue(intake.setStateCommand(IntakeStates.STOWED));
		}
		RobotStates.actionJiggleIntake.whileTrue(intake.setStateCommand(IntakeStates.JIGGLINGUP));
		RobotStates.actionReverseIntake.whileTrue(intake.setStateCommand(IntakeStates.REVERSING));

		// aim turret (hold)
		RobotStates.actionAimTurretHold.whileTrue(turret.setStateCommand(TurretStates.SCORING));

		// aim turret (toggle)
		RobotStates.actionAimTurretToggle.toggleOnTrue(turret.setStateCommand(TurretStates.SCORING));

		RobotStates.actionResetTurret.onTrue(new InstantCommand(() -> turret.resetMotorPosition()));

		// climber
		// RobotStates.actionClimberUp.whileTrue(climber.setStateCommand(ClimberState.RAISING));
		// RobotStates.actionClimberDown.whileTrue(climber.setStateCommand(ClimberState.LOWERING));
	}

	/** rebinds actions to match one driver controls */
	public void bind1Driver() {

		Swerve swerve = RobotContainer.getSwerve();
		Shooter shooter = RobotContainer.getShooter();
		Indexer indexer = RobotContainer.getIndexer();
		Hopper hopper = RobotContainer.getHopper();
		Intake intake = RobotContainer.getIntake();
		Turret turret = RobotContainer.getTurret();
		Operator operator = RobotContainer.getOperator();

		RobotStates.actionResetGyro = operator.start.and(RobotStates.teleop);

		RobotStates.actionShoot = operator.rightTriggerGreater(operatorDeadband).and(RobotStates.teleop);
		RobotStates.actionIndexerShoot = RobotStates.actionShoot.and(RobotStates.shooterGoodToShoot);
		RobotStates.actionSpeedClamp = RobotStates.actionShoot;

		RobotStates.actionDecrementShooter = operator.leftDpad.and(RobotStates.teleop);
		RobotStates.actionIncrementShooter = operator.rightDpad.and(RobotStates.teleop);

		RobotStates.actionReverseShooter = RobotStates.actionIndexerReverse;

		RobotStates.actionIntakeDown = operator.leftTriggerGreater(operatorDeadband).and(RobotStates.teleop);
		RobotStates.actionReverseIntake = operator.leftBumper.and(RobotStates.teleop);
		RobotStates.actionJiggleIntake = RobotStates.actionIndexerShoot.and(RobotStates.actionIntakeDown.negate());

		bind2Driver();
		
		RobotStates.actionStowIntake.toggleOnTrue(intake.setStateCommand(IntakeStates.STOWED));
	}
}
