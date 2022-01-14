package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Base extends SubsystemBase {
    private TalonFX leftBackMotor, leftMidMotor, leftFrontMotor, rightBackMotor, rightMidMotor, rightFrontMotor;

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
        leftBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    @Override
    public void periodic() {
        
    }
  
    @Override
    public void simulationPeriodic() { }

    public void move(double leftSpeed, double rightSpeed) {
        leftBackMotor.set(ControlMode.PercentOutput, leftSpeed);
        rightBackMotor.set(ControlMode.PercentOutput, rightSpeed);
    }

    public double getLeftEncoder() {
        return leftBackMotor.getSelectedSensorPosition();
    }

    public double getRightEncoder() {
        return rightBackMotor.getSelectedSensorPosition();
    }

    public void zeroEncoders() {
        leftBackMotor.setSelectedSensorPosition(0);
        rightBackMotor.setSelectedSensorPosition(0);
    }
}
