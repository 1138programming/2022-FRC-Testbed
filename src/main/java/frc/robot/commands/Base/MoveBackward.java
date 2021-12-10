// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import frc.robot.Robot;
import frc.robot.subsystems.Base;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.time.StopWatch;

/** An example command that uses an example subsystem. */
public class MoveBackward extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  StopWatch timer;
  /**
   * Creates a new MoveForward.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveBackward() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new StopWatch();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.base.move(-10, -10);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.base.move(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.getDuration() >= 10) {
        return true;
    }
    return false;
  }
}
