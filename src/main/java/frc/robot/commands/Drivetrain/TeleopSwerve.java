package frc.robot.commands.Drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.DrivetrainConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Swerve;
import java.util.function.DoubleSupplier;

/** This is the main command to drive the robot */
public class TeleopSwerve extends Command {
	private Swerve m_Swerve;
	private DoubleSupplier translationSup;
	private DoubleSupplier strafeSup;
	private DoubleSupplier rotationSup;

	public TeleopSwerve(
			DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
		// get all values to drive the robot (x,y,z)
		this.translationSup = translationSup;
		this.strafeSup = strafeSup;
		this.rotationSup = rotationSup;
	}

	@Override
	public void initialize() {
		m_Swerve = RobotContainer.getSwerve();
		addRequirements(m_Swerve);
	}

	@Override
	public void execute() {
		// translation, strafe, rotation is the order
		double[] DriverInput = {
			-translationSup.getAsDouble(), -strafeSup.getAsDouble(), -rotationSup.getAsDouble()
		};
		if (!m_Swerve.getFieldRelative()) {
			DriverInput[0] *= 0.5;
			DriverInput[1] *= 0.5;
			DriverInput[2] *= 0.5;
		}

		// you must negate the translation because FRC coordinates and Joystick axis values are
		// opposite to each other. This is done in robotContainer
		Translation2d translation2d = new Translation2d();
		if (m_Swerve.getFieldRelative()) {
			if (DriverStation.getAlliance().get() == Alliance.Blue) {
				translation2d =
						new Translation2d(DriverInput[0], DriverInput[1])
								.times(maxVelocity.in(MetersPerSecond));
			} else {
				translation2d =
						new Translation2d(-DriverInput[0], -DriverInput[1])
								.times(maxVelocity.in(MetersPerSecond));
			}
		} else {
			translation2d =
					new Translation2d(DriverInput[0], DriverInput[1]).times(maxVelocity.in(MetersPerSecond));
		}

		m_Swerve.drive(translation2d, DriverInput[2] * maxAngularVelocity.in(RadiansPerSecond));
	}
}
