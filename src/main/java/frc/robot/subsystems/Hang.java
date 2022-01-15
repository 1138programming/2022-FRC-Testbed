// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
    private TalonSRX left;
    private TalonSRX right;
    private TalonSRX middle;

  /** Creates a new ExampleSubsystem. */
  public Hang() {
      right = new TalonSRX(KRightPort);
      left = new TalonSRX(KLeftPort);
      middle = new TalonSRX(KMiddlePort);
    right.setInverted(true);
    
  }
   public void move(double leftSpeed, double rightSpeed) {
    left.set(ControlMode.PercentOutput, leftSpeed);
    right.set(ControlMode.PercentOutput, rightSpeed);
   }
   public void move2(double middleSpeed){
       middle.set(ControlMode.PercentOutput, middleSpeed);
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
