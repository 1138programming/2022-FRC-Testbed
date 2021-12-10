// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX; 
import com.ctre.phoenix.motorcontrol.ControlMode; 

public class Base extends SubsystemBase {
  private TalonFX leftMotor, rightMotor;

  public Base() {
    leftMotor = new TalonFX(1);
    rightMotor = new TalonFX(2);
  }
  public void move(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
    rightMotor.set(ControlMode.PercentOutput, speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
} 
