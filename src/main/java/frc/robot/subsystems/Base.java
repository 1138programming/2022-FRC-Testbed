package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Base extends SubsystemBase{
  
  private TalonFX left_Front_Motor;
  private TalonFX left_Front_Turn_Motor;
  private TalonFX left_Back_Motor;
  private TalonFX left_Back_Turn_Motor;
  private TalonFX right_Front_Motor;
  private TalonFX right_Front_Turn_Motor;
  private TalonFX right_Back_Motor;
  private TalonFX right_Back_Turn_Motor;
  private double v;
  
  public Base(){
    left_Front_Motor = new TalonFX(1);
    left_Front_Turn_Motor =  new TalonFX(2);
    left_Back_Motor = new TalonFX(3);
    left_Back_Turn_Motor = new TalonFX(4);
    right_Front_Motor = new TalonFX(5);
    right_Front_Turn_Motor = new TalonFX(6);
    right_Back_Motor = new TalonFX(7);
    right_Back_Turn_Motor = new TalonFX(8);
  }

  public void move(double speedl, double speedr){
    left_Front_Motor.set(ControlMode.PercentOutput, speedl);
    left_Front_Turn_Motor.set(ControlMode.PercentOutput, speedl);
    left_Back_Motor.set(ControlMode.PercentOutput, speedl);
    left_Back_Turn_Motor.set(ControlMode.PercentOutput, speedl);
    right_Front_Motor.set(ControlMode.PercentOutput, speedr);
    right_Front_Turn_Motor.set(ControlMode.PercentOutput, speedr);
    right_Back_Motor.set(ControlMode.PercentOutput, speedr);
    right_Back_Turn_Motor.set(ControlMode.PercentOutput, speedr);
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