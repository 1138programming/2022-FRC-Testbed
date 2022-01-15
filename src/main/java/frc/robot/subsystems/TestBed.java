// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestBed extends SubsystemBase {
    private TalonFX left;
    private TalonFX right;

  /** Creates a new ExampleSubsystem. */
  public TestBed() {
      right = new TalonFX(KRightPort);
      left = new TalonFX(KLeftPort);

    
  }
   public void move(double leftSpeed, double rightSpeed) {
    left.set(ControlMode.PercentOutput, leftSpeed);
    right.set(ControlMode.PercentOutput, rightSpeed);
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
