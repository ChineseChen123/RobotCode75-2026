// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Drivetrain.controllers.ChezyController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChezyPose extends Command {
	private final Swerve m_Swerve;

	private Pose2d targetPose2d = null;

	private boolean holdPose;
	private final ChezyController m_ChezyController;

	public ChezyPose(Swerve swerve, ChezyController controller, Pose2d pose, boolean hold) {
		m_Swerve = swerve;
		m_ChezyController = controller;
		targetPose2d = pose;
		holdPose = hold;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_Swerve);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (targetPose2d == null) {
			// FieldPose nearestPose =
			//     new FieldPose(
			//         Alliance.Blue, PeddieBounds.nearestElement(m_Swerve.getPose()), Offset.LEFT);
			// targetPose2d = PeddieBounds.fieldElementToPose2d(m_Swerve, targetPose);
		}
		m_ChezyController.reset(targetPose2d);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		ChassisSpeeds speeds = m_ChezyController.update(targetPose2d); // m_swerve.getPose()
		m_Swerve.setFieldRelative(speeds);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_Swerve.setFieldRelative(new ChassisSpeeds(0, 0, 0));
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_ChezyController.isFinished() && !holdPose;
	}
}
