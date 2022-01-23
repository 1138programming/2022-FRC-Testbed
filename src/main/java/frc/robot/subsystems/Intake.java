package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  // private CANSparkMax intakeMotor;
  private TalonSRX intakeMotor;

  public Intake() {
    // intakeMotor = new CANSparkMax(KIntakeSpark, MotorType.kBrushless);
    // intakeMotor = new TalonSRX(11);  //for proto testing
  }

  public void move(double speed) {
    // intakeMotor.set(ControlMode.PercentOutput, speed);
  }
}
