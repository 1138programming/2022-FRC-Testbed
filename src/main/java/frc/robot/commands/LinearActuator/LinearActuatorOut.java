// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LinearActuator;

import frc.robot.Robot;
import frc.robot.subsystems.LinearActuator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LinearActuatorOut extends CommandBase {
  private final LinearActuator linearActuator;
  private double position;

  /** Creates a new LinearActuator. */
  public LinearActuatorOut(LinearActuator linearActuator) {
    this.linearActuator = linearActuator;
    position = 1;

    addRequirements(linearActuator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.robotContainer.xboxBtnA.get())  {
      position = 1;
    }
    else if (Robot.robotContainer.xboxBtnB.get()) {
      position = 0.75;
    }
    linearActuator.move(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
