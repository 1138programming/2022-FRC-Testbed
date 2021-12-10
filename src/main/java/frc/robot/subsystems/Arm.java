package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Arm extends SubsystemBase {
    private TalonFX leftArmMotor, rightArmMotor;

    public Arm() {
        leftArmMotor = new TalonFX(11);
        rightArmMotor = new TalonFX(12);

        rightArmMotor.setInverted(true);
        rightArmMotor.follow(leftArmMotor);
    }

    @Override
    public void periodic() { }

  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    public void move(double speed) {
        leftArmMotor.set(ControlMode.PercentOutput, speed);
    }
}
