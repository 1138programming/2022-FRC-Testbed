// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import frc.robot.Robot;
import frc.robot.subsystems.NeoBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveWithJoysticks extends CommandBase {

  private final NeoBase base;

  private double xSpeed;
  private double ySpeed;
  private double rot;

  private SlewRateLimiter xSpeedLimiter;
  private SlewRateLimiter ySpeedLimiter;
  private SlewRateLimiter rotLimiter;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(NeoBase base) {

    this.base = base;
  
    xSpeedLimiter = new SlewRateLimiter(6);
    ySpeedLimiter = new SlewRateLimiter(6);
    rotLimiter = new SlewRateLimiter(6);

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = xSpeedLimiter.calculate(Robot.robotContainer.getLogiLeftXAxis());

    ySpeed = ySpeedLimiter.calculate(Robot.robotContainer.getLogiLeftYAxis());

    rot = rotLimiter.calculate(Robot.robotContainer.getLogiRightXAxis());
    
    base.drive(xSpeed, ySpeed, rot, true);
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
