// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.EndEffectorConstants.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.controls.VelocityVoltage;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public static enum ShooterStates {
    PLACEHOLDER,
    PLACEHOLDER2
  }

  private final TalonFX m_innerMotor;
  private final TalonFX m_outerMotor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  
  
  public Shooter() {
    m_outerMotor = new TalonFX(outerMotorCANID);
    m_innerMotor = new TalonFX(innerMotorCANID);
  }
  public void setOuter(double velocity){
    m_outerMotor.setControl(velocityRequest.withVelocity(velocity));
  }
  public void setInner(double velocity){
    m_innerMotor.setControl(velocityRequest.withVelocity(velocity));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
