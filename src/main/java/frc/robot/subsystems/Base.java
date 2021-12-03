// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX; //do proper offline install https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/tag/v5.19.4.1
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Base extends SubsystemBase {
  /** Creates a new Base. */
  private final TalonFX falcon; 

  public Base() {
    falcon = new TalonFX(KfalconPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
  * Function Moves the base directly
  * @Category: Base
  *  
  * @param PWM   Speed to move the left side at
  */
  public void move(int PWM) {
    falcon.set(ControlMode.PercentOutput, PWM);
  }
}