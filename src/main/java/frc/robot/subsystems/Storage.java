package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;;

public class Storage {
  private CANSparkMax storageMotor;
    
  public Storage(){
    // storageMotor = new CANSparkMax(KStorageSpark, MotorType.kBrushless);
  }

  public void move(double speed) {
    // storageMotor.set(speed);
  }
}