package frc.robot.commands.Base;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;

public class moveEncoder extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private double distance;
    private int speed;

    public moveEncoder(int speed, double distance) {
      this.distance = distance;
      this.speed = speed;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(Robot.base);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      Robot.base.move(speed);
      Robot.base.zeroEncoders();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if (Math.abs(Robot.base.getEncoders()) >= Math.abs(distance)) {
        Robot.base.move(0);
        return true;
      }
      else {
        return false;
      }
    }
  }
