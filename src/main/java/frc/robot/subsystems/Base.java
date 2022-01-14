// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


public class Base extends SubsystemBase {
  //top left
  private TalonFX topLeft;
  //top right
  private TalonFX topRight;
  //bottom left
  private TalonFX bottomLeft;
  //bottom right
  private TalonFX bottomRight;

  private double value;
  public Base() {
    topLeft = new TalonFX(69);
    topLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 500);
    topRight = new TalonFX(420);
    topRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 500);
    bottomLeft = new TalonFX(21);
    bottomLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 500);
    bottomRight = new TalonFX(82);
    bottomRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 500);
  }
  public void move(int speed) {
    topLeft.set(ControlMode.PercentOutput, speed);
    topRight.set(ControlMode.PercentOutput, -speed);
    bottomLeft.set(ControlMode.PercentOutput, speed);
    bottomRight.set(ControlMode.PercentOutput, -speed);
  }
  public void turn(int speed) {
    topLeft.set(ControlMode.PercentOutput, speed);
    topRight.set(ControlMode.PercentOutput, speed);
    bottomLeft.set(ControlMode.PercentOutput, speed);
    bottomRight.set(ControlMode.PercentOutput, speed);
  }
  public void zeroEncoders() {
    topLeft.setSelectedSensorPosition(0,0,0);
    topRight.setSelectedSensorPosition(0,0,0);
    bottomLeft.setSelectedSensorPosition(0,0,0);
    bottomRight.setSelectedSensorPosition(0,0,0);
  }
  public double getEncoders() {
    value = (topLeft.getSelectedSensorPosition() + -topRight.getSelectedSensorPosition() + bottomLeft.getSelectedSensorPosition() + -bottomRight.getSelectedSensorPosition())/4;
    return value;
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
