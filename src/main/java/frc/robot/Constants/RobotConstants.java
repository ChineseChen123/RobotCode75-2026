// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;



/**
 * This class is meant ry.TrapezoidProflie;
import edu.first.first.math.etil.Units;
to house the configs for specific motors All configs from CTRE motors are
 * unit-aware, especially configs for closed loop gains timeSync can only be used on a CANivore any
 * TorqueCurrentFOC gains/control modes can only be used with Phoenix pro (HIGHLY RECCOMENDED TO
 * USE)
 */
public final class RobotConstants {
	public static final String superstructureCANBusName = "Superstructure";
	public static final boolean TUNING_MODE = true; // Set to true for tunable numbers
}
