//it doesn't work
package frc.robot.commands.Base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class drive_with_joystick extends CommandBase{
    
    public drive_with_joystick(){
        addRequirements(Robot.base);
    }

    private double quadraticCurve(double input){
        if(input > 0){
            input *= input;
        }
        else{
            input *= -input;
        }
        return input;
    }

   /* private double cubicCurve(double input){
        return input * input * input;
    }

    private double quarticCurve(double input){
        if(input > 0){
            input *= input;
            input *= input;
        }
        else{
            input *= input;
            input *= -input;
        }
        return input;
    }*/

    @Override

    public void execute(){
        double leftPWM = 0;
        double rightPWM = 0;

        if(Robot.autonomousActive == false){
            double leftY = quadraticCurve(Robot.m_robotContainer.getLeftAxis());
            double rightX = Robot.m_robotContainer.getArcadeRightAxis() * 0.4;
            leftPWM = leftY - rightX;
            rightPWM  =leftY + rightX;
        }

        Robot.base.move(leftPWM, rightPWM);
    }
        @Override
        public void end(boolean Interrupted){
        }

        @Override
        public boolean isFinished(){
            return false;
        }
    
}
