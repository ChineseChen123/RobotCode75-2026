// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffectors;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.RobotConstants.*;
import static frc.robot.Constants.EndEffectorConstants.MotorConfigs.*;
import static frc.robot.Constants.EndEffectorConstants.*;
import static edu.wpi.first.units.Units.Rotations;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;

public class Intake extends SubsystemBase {

  public static enum IntakeStates {
    DEFAULT,
    INTAKING,
    REVERSING
  }
  
  private TalonFX m_IntakeMotor;
  private TalonFX m_PivotMotor; 

  private final DutyCycleEncoder m_absoluteEncoder;

  private IntakeStates m_IntakeState;

  //

  /** Creates a new Intake. */
  public Intake() {
      m_IntakeMotor = new TalonFX(intakeMotorCanID, superstructureCANBusName);
      m_PivotMotor = new TalonFX(pivotCanID, superstructureCANBusName);
      m_absoluteEncoder = new DutyCycleEncoder(pivotEncoderPort, 1, pivotZeroPoint.in(Rotations));
		  m_PivotMotor.getConfigurator().apply(getPivotConfiguration());
      m_IntakeState = IntakeStates.DEFAULT;
    }

  
  @Logged (key = "Intake State", importance = Importance.CRITICAL)
	public IntakeStates getIntakeState() {
		return m_IntakeState;
	}

  public void setState(IntakeStates state) {
		m_IntakeState = state;
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}