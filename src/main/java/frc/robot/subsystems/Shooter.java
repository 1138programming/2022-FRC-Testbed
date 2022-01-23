package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;;

public class Shooter {
  private CANSparkMax shooterMotor;

  public Shooter() {
    // shooterMotor = new CANSparkMax(KShooterSpark, MotorType.kBrushless);
  }

  public void move(double speed) {
    // shooterMotor.set(speed);
  }
}
