// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Base extends SubsystemBase {
    private TalonFX leftBackMotor, leftMidMotor, leftFrontMotor, rightBackMotor, rightMidMotor, rightFrontMotor;
  /** Creates a new ExampleSubsystem. */
  public Base() {
        leftBackMotor = new TalonFX(1);
        leftMidMotor = new TalonFX(2);
        leftFrontMotor = new TalonFX(3);
        rightBackMotor = new TalonFX(4);
        rightMidMotor = new TalonFX(5);
        rightFrontMotor = new TalonFX(6);
        rightBackMotor.setInverted(true);
        rightMidMotor.follow(rightBackMotor);
        rightFrontMotor.follow(rightBackMotor);
        leftMidMotor.follow(leftBackMotor);
        leftFrontMotor.follow(leftBackMotor);
        rightBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegradedSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

    public void move()
    {
        leftFrontMotor.set(ControlMode.PercentOutput, leftSpeed);
        rightFrontMotor.set(ControlMode.PercentOutput, rightSpeed);
        leftBackMotor.set(ControlMode.PercentOutput, leftSpeed);
        rightBackMotor.set(ControlMode.PercentOutput, rightSpeed);
        leftMidMotor.set(ControlMode.PercentOutput, leftSpeed);
        rightMidMotor.set(ControlMode.PercentOutput, rightSpeed);
    }

}
