package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Base extends SubsystemBase {
    private TalonFX leftMotor, rightMotor;

    public Base() {
        leftMotor = new TalonFX(0);
        rightMotor = new TalonFX(10);
    }

    @Override
    public void periodic() {
        leftMotor.DestroyObject();
        rightMotor.DestroyObject();
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}
