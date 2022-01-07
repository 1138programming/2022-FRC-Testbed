package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Base extends SubsystemBase {
    private TalonFX leftMotor, rightMotor, nonProgrammer;

    public Base() {
        leftMotor = new TalonFX(1);
        rightMotor = new TalonFX(2);
        
        rightMotor.setInverted(true);

        rightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        nonProgrammer = new TalonFX(3);
    }

    @Override
    public void periodic() {
        nonProgrammer.DestroyObject();
    }
  
    @Override
    public void simulationPeriodic() { }

    public void move(double leftSpeed, double rightSpeed) {
        leftMotor.set(ControlMode.PercentOutput, leftSpeed);
        rightMotor.set(ControlMode.PercentOutput, rightSpeed);
    }

    public double getLeftEncoder() {
        return leftMotor.getSelectedSensorPosition();
    }

    public double getRightEncoder() {
        return rightMotor.getSelectedSensorPosition();
    }

    public void zeroEncoders() {
        leftMotor.setSelectedSensorPosition(0);
        rightMotor.setSelectedSensorPosition(0);
    }
}
