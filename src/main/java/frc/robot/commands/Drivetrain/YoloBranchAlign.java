package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Drivetrain.controllers.YoloController;

public class YoloBranchAlign extends Command {
	private final Swerve m_Swerve;
	private final YoloController m_YoloController;
	private boolean isAlignInPlace = false;

	public YoloBranchAlign(Swerve swerve, boolean alignInPlace) {
		m_Swerve = swerve;
		isAlignInPlace = alignInPlace;
		m_YoloController = m_Swerve.getYoloController();
		addRequirements(m_Swerve);
	}

	@Override
	public void initialize() {
		m_YoloController.reset(isAlignInPlace);
	}

	@Override
	public void execute() {
		System.out.println("yoloing " + Timer.getFPGATimestamp());
		ChassisSpeeds speeds = m_YoloController.update();
		// speeds.vyMetersPerSecond = -speeds.vyMetersPerSecond;
		m_Swerve.setRobotRelative(speeds);
	}

	@Override
	public boolean isFinished() {
		return m_YoloController.atGoal();
	}

	@Override
	public void end(boolean interrupted) {
		// if (interrupted) {
		//   System.out.println("Ended, interrupted");
		// } else {
		//   System.out.println("ended, not interrupted");
		// }
		m_Swerve.stopModules();
	}
}
