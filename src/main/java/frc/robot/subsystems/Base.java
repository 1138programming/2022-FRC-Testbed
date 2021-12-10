// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;



public class Base extends SubsystemBase {
  //top left
  private TalonFX humzah1;
  //top right
  private TalonFX humzah2;
  //bottom left
  private TalonFX humzah3;
  //bottom right
  private TalonFX humzah4;
  public Base() {
    humzah1 = new TalonFX(69);
    humzah2 = new TalonFX(420);
    humzah3 = new TalonFX(21);
    humzah4 = new TalonFX(82);
  }
  public void move(int speed) {
    humzah1.set(ControlMode.PercentOutput, speed);
    humzah2.set(ControlMode.PercentOutput, -speed);
    humzah3.set(ControlMode.PercentOutput, speed);
    humzah4.set(ControlMode.PercentOutput, -speed);
  }
  public void turn(int speed) {
    humzah1.set(ControlMode.PercentOutput, speed);
    humzah2.set(ControlMode.PercentOutput, speed);
    humzah3.set(ControlMode.PercentOutput, speed);
    humzah4.set(ControlMode.PercentOutput, speed);
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
