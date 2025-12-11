// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DrivetrainConstants.ControllerConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.ObjectDetetectorCamera;

public class YoloController {
	// private final TunableNumber[] strafePID = {
	//   new TunableNumber("YOLO Align/P", 0.035),
	//   new TunableNumber("YOLO Align/I", 0),
	//   new TunableNumber("YOLO Align/D", 0.002),
	//   new TunableNumber("YOLO Align/Tolerance", 0.25)
	// };

	private Swerve m_swerve;
	private final ObjectDetetectorCamera m_BranchDetectorCamera;
	private boolean isAlignInPlace;
	private ChassisSpeeds desiredSpeeds;

	private final PIDController yController;
	private double yCommand;

	private final double finalYawSetpointDegrees = 0; // -1.1
	private final double driveIntoReefSpeed = .75;
	double startTime = -1;

	/** Creates a new YoloController. */
	public YoloController() {
		// initialize swerve and controllers
		m_swerve = RobotContainer.getSwerve();
		m_BranchDetectorCamera = RobotContainer.getBranchCamera();
		desiredSpeeds = new ChassisSpeeds();
		yController =
				new PIDController(
						ControllerConstants.VisionAlign.xP,
						ControllerConstants.VisionAlign.xI,
						ControllerConstants.VisionAlign.xD);
		yController.setTolerance(.2);
		yController.setSetpoint(finalYawSetpointDegrees);
	}

	/** set PIDs based on align method */
	/** align in place = true: when we are already bumpers up */
	/** align in place = false: we need to drive forward into the reef */
	public void reset(boolean alignInPlace) {
		if (m_swerve == null) {
			m_swerve = RobotContainer.getSwerve();
		}
		isAlignInPlace = alignInPlace;
		if (isAlignInPlace) {
			yController.setP(.025);
			yController.setD(0.002);
			yController.setTolerance(2.5);
			// yController.setP(strafePID[0].getNumber());
			// yController.setI(strafePID[1].getNumber());
			// yController.setD(strafePID[2].getNumber());
			// yController.setTolerance(strafePID[3].getNumber());
		} else {
			// yController.setP(strafePID[0].getNumber());
			// yController.setI(strafePID[1].getNumber());
			// yController.setD(strafePID[2].getNumber());
			// yController.setTolerance(strafePID[3].getNumber());
			yController.setP(0.08);
			yController.setD(0.005);
			yController.setTolerance(.025);
		}
		yController.setSetpoint(finalYawSetpointDegrees);
		desiredSpeeds = new ChassisSpeeds(0, 0, 0);
		startTime = -1;
	}

	/** returns speeds robot needs to move at */
	public ChassisSpeeds update() {
		if (m_swerve == null) {
			m_swerve = RobotContainer.getSwerve();
		}
		if (startTime == -1) {
			startTime = Timer.getFPGATimestamp();
		}

		// yController.setP(strafePID[0].getNumber());
		// yController.setI(strafePID[1].getNumber());
		// yController.setD(strafePID[2].getNumber());
		// yController.setTolerance(strafePID[3].getNumber());

		m_BranchDetectorCamera.updateByUnreadResults();

		if (!isAlignInPlace) {
			if (!m_BranchDetectorCamera.hasTargets()) {
				if (!desiredSpeeds.equals(new ChassisSpeeds(0, 0, 0))) { // not fresh command
					desiredSpeeds.vyMetersPerSecond =
							desiredSpeeds.vyMetersPerSecond * .75; // should move in same direction but slower
				} else {
					desiredSpeeds = new ChassisSpeeds(driveIntoReefSpeed, 0, 0);
				}
			} else {

				double targetYaw = m_BranchDetectorCamera.getTargetYaw(0).getAsDouble();

				yCommand = yController.calculate(targetYaw);

				desiredSpeeds.vxMetersPerSecond = driveIntoReefSpeed;
				desiredSpeeds.vyMetersPerSecond = yCommand;
			}
		} else {
			if (!m_BranchDetectorCamera.hasTargets()) {
				desiredSpeeds = new ChassisSpeeds(0, 0.0, 0); // scoot toward direction of last seen target
			} else {
				double targetYaw = m_BranchDetectorCamera.getTargetYaw(0).getAsDouble();

				yCommand = yController.calculate(targetYaw);
				desiredSpeeds.vxMetersPerSecond = 0; // can't push into reef while trying to align
				desiredSpeeds.vyMetersPerSecond = yCommand;
			}
		}
		return desiredSpeeds;
	}

	/** return last left/right velocity calculated */
	public double getCommand() {
		return yCommand;
	}

	/** returns whether we are done aligning */
	public boolean atGoal() {
		// should be stalling when driving into reef
		// return Timer.getFPGATimestamp() - startTime >= 0.5;
		// actual vx less than stall speed

		if (!isAlignInPlace) {
			return false;
			// return m_Swerve.getChassisSpeeds().vxMetersPerSecond <= stallSpeedThreshold
			//     && (Timer.getFPGATimestamp() - startTime) >= 0.2;
		} else {
			return yController.atSetpoint() || getCommand() < 0.04;
		}
	}
}
