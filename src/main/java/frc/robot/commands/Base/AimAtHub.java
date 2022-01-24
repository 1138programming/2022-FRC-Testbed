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

public class AimAtHub extends CommandBase {
private NeoBase base;
private Camera camera;

private double distanceOffset;

private double rot;
private double xSpeed;
private double ySpeed;

  /** Creates a new AimAtHub. */
  public AimAtHub() {
    this.base = base;
    this.camera = camera;

    rot = 0; //temporary #
    xSpeed = 0;
    ySpeed = 0;
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
    distanceOffset = camera.getXOffset();
    if (distanceOffset > kDesiredXOffset) {
      rot = -0.6;
    }
    else if (distanceOffset < kDesiredXOffset) {
      rot = 0.6;
    }

    base.drive(xSpeed, ySpeed, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (distanceOffset >= kDesiredXOffset - kXOffsetDeadzone && distanceOffset <= kDesiredXOffset - kXOffsetDeadzone) {
      return true;
    }
    return false;
  }
}