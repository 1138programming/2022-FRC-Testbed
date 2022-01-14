package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierConfiguration;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Gains;

public class swerveModule {
    private static double maxDeltaTicks = 980.0;

    //to set the number of the motor to make the base motor work better, don't change two of these.
    private static final Gains kDriveGains = new Gains(15, 0.01, 0.1, 0.2, 0, 1.0);
    private static final Gains kAngleGains = new Gains(0.6, 0.0, 0.25, 0.0, 0, 1.0);

    //CANCoder has 4096 ticks per rotation
    private static double kEncoderTicksPerRotation = 4096;

    private double desiredTicks;

    private TalonFX driveMotor;             //set the motor
    private TalonFX angleMotor;             //set the motor
    private CANifier canifier;              //don't touch it
    private Rotation2d offset;              //don't touch it

    //create the constructor
    public swerveModule(TalonFX driveMotor, TalonFX angleMotor, CANifier canifier, Rotation2d offset){
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.canifier = canifier;
        this.offset  = offset;

        //form: angleMotor.configAllowableClosedloopError(0, 0, 0)
        //all of the config_kP/config_kI/config_kD are came from 'com.ctre.phoenix.motorcontrol.can' this path
        angleMotor.config_kP(0, kAngleGains.kP, 0);
        angleMotor.config_kI(0,kAngleGains.kI, 0);
        angleMotor.config_kD(0, kAngleGains.kD, 0);

        //to keep the robot stopped when you want it stopped, it's not necessary but useful
        //comes from the same place as the confic_k*
        angleMotor.setNeutralMode(NeutralMode.Brake);

        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

        //driveMotor.configAllowableClosedloopError(0, 0, 0)
        driveMotor.config_kF(0, kDriveGains.kF,0);
        driveMotor.config_kP(0, kDriveGains.kP, 0);
        driveMotor.config_kI(0, kDriveGains.kI, 0);
        driveMotor.config_kD(0, kDriveGains.kD, 0);

        driveMotor.setNeutralMode(NeutralMode.Brake);
    }

    public Rotation2d getAngle(){
        double deg = (canifier.getQuadraturePosition() % ticksPerRevolution) * 360 / tickPerRevolution;
    }
}