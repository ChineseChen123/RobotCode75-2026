// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import static frc.robot.Constants.VisionConstants.*;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class Limelight extends SubsystemBase {

	private boolean isIMUReset = false;

	public Limelight() {
		// set valid tags for pose estimation
		LimelightHelpers.SetFiducialIDFiltersOverride(llName, validMT2Tags);

		// set camera pose relative to robot center
		LimelightHelpers.setCameraPose_RobotSpace(llName, LimelightPose.getX(), LimelightPose.getY(), LimelightPose.getZ(),
				LimelightPose.getRotation().getX(), LimelightPose.getRotation().getY(),
				LimelightPose.getRotation().getZ());
	}

	public void resetInternalIMU() {
		LimelightHelpers.SetIMUMode(llName, 1);
		LimelightHelpers.SetRobotOrientation(llName, RobotContainer.getSwerve().getHeading().getDegrees(), 0, 0, 0, 0, 0);
		LimelightHelpers.SetIMUMode(llName, 2);
		System.out.println("Limelight IMU reset");
		isIMUReset = true;
	}

	public LimelightHelpers.PoseEstimate getEstimatedPose() {
		if (!isIMUReset) {
			resetInternalIMU();
		}
		LimelightHelpers.PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
		if (mt2Estimate == null || mt2Estimate.tagCount == 0) {
			return null;
		}
		LimelightHelpers.RawFiducial[] detectedTags = mt2Estimate.rawFiducials;
		for (LimelightHelpers.RawFiducial tag : detectedTags) {
			// discard result if tag is too far or too small
			if (tag.distToRobot > maxTagDistanceThreshold || tag.ta < minTagAreaThreshold) {
				return null;
			}
			// discard result if too ambiguous
			if (mt2Estimate.tagCount == 1 && tag.ambiguity > ambiguityThreshold) {
				return null;
			}
			if (mt2Estimate.tagCount > 1 && tag.ambiguity > multiTagAmbiguityThreshold) {
				return null;
			}
		}
		return mt2Estimate;
	}

	@Override
	public void periodic() {

	}
}
