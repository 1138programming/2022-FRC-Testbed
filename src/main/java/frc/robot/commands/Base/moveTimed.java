package frc.robot.commands.Base;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import edu.wpi.first.wpilibj.Timer;

public class moveTimed extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Timer timer;
    private double time;
    private int speed;

    public moveTimed(int speed, double time) {
      timer = new Timer();
      this.time = time;
      this.speed = speed;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(Robot.base);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      timer.start();
      Robot.base.move(speed);
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
      if (timer.hasElapsed(time)) {
        timer.stop();
        timer.reset();
        Robot.base.move(0);
        return true;
      }
      else {
        return false;
      }
    }
  }
