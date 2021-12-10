// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import frc.robot.subsystems.Base;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.time.StopWatch;

/** An example command that uses an example subsystem. */
public class MoveBackward extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Base m_base;

  /**
   * Creates a new MoveForward.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveBackward(Base base) {
    m_base = base;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    StopWatch timer = new StopWatch();
    timer.start();
    while (timer.getDuration() < 10) {
        m_base.move(-10, -10);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_base.move(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
