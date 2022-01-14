package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController; 

import edu.wpi.first.wpilibj2.command.SubsystemBase; 

public class Base extends SubsystemBase {

  private TalonFX frontLeft;
  private TalonFX frontRight;
  private TalonFX backLeft;
  private TalonFX backRight;
  private final PIDController motorController;
  
  
  public Base() {
  //Top Motors
  frontLeft = new TalonFX(1);
  frontRight = new TalonFX(2);

  //Bottom Motors
  backLeft = new TalonFX(3);
  backRight = new TalonFX(4);

  //Configuring Encoders
  bottomRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 500);
  bottomLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 500);

  //Making Motors into indentured servant
  backLeft.follow(frontLeft);
  backRight.follow(frontRight);

  //Creating PID controller
  //Will add values later
  motorController = new PIDController(Kp, Ki, Kd, source, output);
  
  public void move(double leftMotors, double rightMotors) {
    frontLeft.set(ControlMode.PercentOutput, leftMotors);
    frontRight.set(ControlMode.PercentOutput, rightMotors);
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
