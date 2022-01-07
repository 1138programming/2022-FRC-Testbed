// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import frc.robot.Robot;
import frc.robot.subsystems.Base;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveBaseFor extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  boolean direction;
  double length;
  double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveBaseFor(boolean direction, double length, double speed) {
    this.direction = direction;
    this.length = length;
    this.speed = speed;

    addRequirements(Robot.base);
  }

  public MoveBaseFor() {
    addRequirements(Robot.base);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.base.zeroEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (direction == false) {
      Robot.base.move(-speed, -speed);
    }
    else {
      Robot.base.move(speed, speed);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.base.move(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.base.getRightEncoder() >= length || Robot.base.getLeftEncoder() >= length) {
      return true;
    }
    return false;
  }
}

