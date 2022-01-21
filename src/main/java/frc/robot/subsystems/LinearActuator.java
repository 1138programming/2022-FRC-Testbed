package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearActuator extends SubsystemBase {
  private Servo linearActuator;


  public LinearActuator() {
    linearActuator = new Servo(KLinearActuator);
    linearActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

  public void move(double position) {
    linearActuator.set(position);
  }
}
