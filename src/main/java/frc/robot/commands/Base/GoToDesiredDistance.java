// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import frc.robot.Robot;
import frc.robot.subsystems.NeoBase;
import frc.robot.subsystems.Camera;
import static frc.robot.Constants.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GoToDesiredDistance extends CommandBase {
  private NeoBase base;
  private Camera camera;

  private double distanceOffset;

  private double xSpeed;
  private double ySpeed;
  private double rot;

  /** Creates a new goToDesiredDistance. */
  public GoToDesiredDistance(NeoBase base, Camera camera) {
    this.base = base;
    this.camera = camera;

    xSpeed = 0;
    ySpeed = 0;
    rot = 0;

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceOffset = camera.getYOffset();
    if (distanceOffset > kDesiredYOffset) {
      ySpeed = -0.6;
    }
    else if (distanceOffset < kDesiredYOffset) {
      ySpeed = 0.6;
    }

    base.drive(xSpeed, ySpeed, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (distanceOffset >= kDesiredYOffset - kYOffsetDeadzone && distanceOffset <= kDesiredYOffset - kYOffsetDeadzone) {
      return true;
    }
    return false;
  }
}