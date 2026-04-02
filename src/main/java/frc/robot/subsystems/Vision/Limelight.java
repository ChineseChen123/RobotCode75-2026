// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.VisionConstants.*;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.LimelightHelpers;
import frc.robot.state.RobotStates;

public class Limelight extends SubsystemBase {

	private final String llName;
	private final Pose3d llPose;

	private boolean isIMUReset = false;

	private double minAmbiguity = -1;

	public Limelight(String name, Pose3d pose) {
		llName = name;
		llPose = pose;

		// set valid tags for pose estimation
		// LimelightHelpers.SetFiducialIDFiltersOverride(llName, validMT2Tags);

		// set camera pose relative to robot center
		LimelightHelpers.setCameraPose_RobotSpace(
				llName,
				llPose.getX(),
				llPose.getY(),
				llPose.getZ(),
				llPose.getRotation().getMeasureX().in(Degrees),
				llPose.getRotation().getMeasureY().in(Degrees),
				llPose.getRotation().getMeasureZ().in(Degrees));
	}

	public void resetInternalIMU() {
		LimelightHelpers.SetIMUMode(llName, 1);
		updateIMU();
		LimelightHelpers.SetIMUMode(llName, 0);
		System.out.println("Limelight IMU reset");
		isIMUReset = true;
	}

	public void updateIMU() {
		LimelightHelpers.SetRobotOrientation(
				llName, RobotStates.robotHeading.get().getDegrees(), 0, 0, 0, 0, 0);
	}

	public LimelightHelpers.PoseEstimate getEstimatedPose() {
		if (!isIMUReset) {
			resetInternalIMU();
		}

		LimelightHelpers.PoseEstimate mt2Estimate =
				LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
		if (mt2Estimate == null || mt2Estimate.tagCount == 0) {
			minAmbiguity = -1;
			return null;
		}
		minAmbiguity = Double.MAX_VALUE;
		LimelightHelpers.RawFiducial[] detectedTags = mt2Estimate.rawFiducials;
		double minDist = Double.MAX_VALUE;
		for (LimelightHelpers.RawFiducial tag : detectedTags) {
			minAmbiguity = Math.min(minAmbiguity, tag.ambiguity);
			minDist = Math.min(minDist, tag.distToRobot);
		}

		// TODO figure out threshold
		// if (minAmbiguity > minAmbiguityThreshold) {
		// 	return null;
		// }

		// if (minDist > 3) {
		// 	return null;
		// }

		return mt2Estimate;
	}

	@Logged(key = "Estimated Pose", importance = Importance.CRITICAL)
	public Pose2d loggedPoseEstimate() {
		LimelightHelpers.PoseEstimate pose = getEstimatedPose();
		if (pose != null) {
			return pose.pose;
		}
		return new Pose2d();
	}

	@Logged(key = "Min Ambiguity", importance = Importance.DEBUG)
	public double minAmbiguity() {
		return minAmbiguity;
	}

	public double getFOM(LimelightHelpers.PoseEstimate pose) {

		if (pose == null) {
			return Double.POSITIVE_INFINITY;
		}

		double fom = 0;

		double linearSpeed = RobotStates.robotSpeedMagnitude.get();
		fom += 0.5 * linearSpeed / DrivetrainConstants.maxVelocity.in(MetersPerSecond);

		double angularSpeed = RobotStates.fieldRelativeSpeeds.get().omegaRadiansPerSecond;
		fom += 0.75 * angularSpeed / DrivetrainConstants.maxAngularVelocity.in(RadiansPerSecond);

		fom *= Math.sqrt(pose.avgTagDist);

		fom += 4 * minAmbiguity;

		return fom / pose.tagCount;
	}

	public double getStdev(LimelightHelpers.PoseEstimate estimate) {
		double linearSpeed = RobotStates.robotSpeedMagnitude.get();
		return estimate != null && estimate.rawFiducials.length > 0 ? 
            2 + Math.pow(Arrays.stream(estimate.rawFiducials).mapToDouble(fiducial -> fiducial.distToCamera).min().getAsDouble(),2) / estimate.tagCount
				+ (linearSpeed > 2 ? Math.pow(linearSpeed, 2) : 2 * linearSpeed) : 
            Double.MAX_VALUE;
	}

	public void setCooling(boolean cooling) {
		LimelightHelpers.SetThrottle(llName, cooling ? 200 : 0);
	}

	@Override
	public void periodic() {
		updateIMU();
	}
}
