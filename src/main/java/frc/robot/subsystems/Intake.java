package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor;

  public Intake() {
    intakeMotor = new CANSparkMax(KIntakeSpark, MotorType.kBrushless);
  }

  public void move(double speed) {
    intakeMotor.set(speed);
  }
}
