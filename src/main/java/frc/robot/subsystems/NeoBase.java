package frc.robot.subsystems;

import static frc.robot.Constants.*;
import frc.robot.Gains;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class NeoBase extends SubsystemBase {
   
  public static AHRS gyro = new AHRS(SPI.Port.kMXP); //not on the bot yet

  private SwerveDriveKinematics kinematics;

  private SwerveX[] modules;

  // private double[] KAbsoluteResetPoints = {3192, 3823, 3220, 2280};
  //offset of each module, in degrees
  private double frontLeftOffset = 178.5;
  private double frontRightOffset = 333.5;
  private double backLeftOffset = 282.8; 
  private double backRightOffset = 12.6;

  public NeoBase() {

    //defining the physical position of the swerve modules
    kinematics = new SwerveDriveKinematics(
      new Translation2d(
        Units.inchesToMeters(14.5),
        Units.inchesToMeters(14.5)
      ),
      new Translation2d(
        Units.inchesToMeters(14.5),
        Units.inchesToMeters(-14.5)
      ),
      new Translation2d(
        Units.inchesToMeters(-14.5),
        Units.inchesToMeters(14.5)
      ),
      new Translation2d(
        Units.inchesToMeters(-14.5),
        Units.inchesToMeters(-14.5)
      )
    );

    //swerve module instances init (in an array)
  modules = new SwerveX[] {
    new SwerveX(new CANSparkMax(frontLeftDriveId, MotorType.kBrushless), new CANSparkMax(frontLeftSteerId, MotorType.kBrushless), new DutyCycleEncoder(frontLeftMagEncoderId), Rotation2d.fromDegrees(frontLeftOffset)), // Front Left
    new SwerveX(new CANSparkMax(frontRightDriveId, MotorType.kBrushless), new CANSparkMax(frontRightSteerId, MotorType.kBrushless), new DutyCycleEncoder(frontRightMagEncoderId), Rotation2d.fromDegrees(frontRightOffset)), // Front Right
    new SwerveX(new CANSparkMax(backLeftDriveId, MotorType.kBrushless), new CANSparkMax(backLeftSteerId, MotorType.kBrushless), new DutyCycleEncoder(backLeftMagEncoderId), Rotation2d.fromDegrees(backLeftOffset)), // Back Left
    new SwerveX(new CANSparkMax(backRightDriveId, MotorType.kBrushless), new CANSparkMax(backRightSteerId, MotorType.kBrushless), new DutyCycleEncoder(backRightMagEncoderId), Rotation2d.fromDegrees(backRightOffset))  // Back Right
  };

  SmartDashboard.putNumber("Base Drive kP", 0.0);
  SmartDashboard.putNumber("Base Drive kI", 0.0);
  SmartDashboard.putNumber("Base Drive kD", 0.0);
  // gyro.reset(); 
  }

  /**
   * Function to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
  
  SwerveModuleState[] states =
    kinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-gyro.getAngle()))
        : new ChassisSpeeds(xSpeed, ySpeed, rot));
  SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxMotorOutput);
  
  //setting module states, aka moving the motors
//   for (int i = 0; i < states.length; i++) {
//     SwerveX module = modules[i];
//     SwerveModuleState state = states[i];
//     module.setDesiredState(state);
//   }

SwerveX module = modules[0];
SwerveModuleState state = states[0];
module.setDesiredState(state);
}

  // public void resetGyro() {
  //   gyro.reset(); //recalibrates gyro offset
  // }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Left Front Deg Angle", modules[0].getAngleDeg());
    SmartDashboard.putNumber("Front Left Absolute Angle", modules[0].getAngleDeg());
    SmartDashboard.putNumber("DriveVel", modules[0].getDriveEncoderVel());
    // SmartDashboard.putNumber("Left Front Raw Encoder", modules[0].getRawAbsoluteTicks());
    // SmartDashboard.putNumber("Right Front Rel Angle", modules[1].getAngleTicks());
    // SmartDashboard.putNumber("Left Back Rel Angle", modules[2].getAngleTicks());
    // SmartDashboard.putNumber("Right Back Rel Angle", modules[3].getAngleTicks());

    setModuleGains(SmartDashboard.getNumber("Base Drive kP", 0.0), SmartDashboard.getNumber("Base Drive kI", 0.0), SmartDashboard.getNumber("Base Drive kD", 0.0));
    // This method will be called once per scheduler run
  }

  public void setModuleGains(double kP, double kI, double kD){
    // modules[0].setDrivePIDGains(kP, kI, kD);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  class SwerveX {

    private final Gains kDriveGains = new Gains(0.1, 0.00, 0, 0.2);

    private final Gains kAngleGains = new Gains(0.006, 0.0, 0.0, 0.0); 

    private double kEncoderTicksPerRotation = 4096;
    public double kWheelDiameterMeters = Units.inchesToMeters(4);
    public double kDriveMotorGearRatio = 1 / 6.55;
    public double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    private double kAngleMotorShaftToWheelRatio = 1 / 10.285714; //1/(72/7)
    private double kAngleEncoderRot2Deg = kAngleMotorShaftToWheelRatio * 360;
    private double kMagEncoderPeriod = 0.04;

    private SwerveModuleState m_desiredState; //for testing

    private CANSparkMax driveMotor;
    private CANSparkMax angleMotor;
    private DutyCycleEncoder magEncoder;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder angleEncoder;
    private PIDController driveController, angleController;
    private Rotation2d offset;
    private double[] pulseWidthAndPeriod = new double[]{1, 1/244}; //pulse width found in mag encoder manual pdf, period is 1/frequency (also found in pdf)

    private double angleMotorOutput;
    
    SwerveX(CANSparkMax driveMotor, CANSparkMax angleMotor, DutyCycleEncoder magEncoder, Rotation2d offset) {
      this.driveMotor = driveMotor;
      this.angleMotor = angleMotor;
      this.magEncoder = magEncoder;
      this.offset = offset;
      
      //PIDControllers:
      driveController = new PIDController(kDriveGains.kP, kDriveGains.kI, kDriveGains.kD);
      angleController = new PIDController(kAngleGains.kP, kAngleGains.kI, kAngleGains.kD);
      
      //telling the pid controller that 360 deg in one direction is the same as 360 deg in the other direction
      angleController.enableContinuousInput(-180, 180);
      
      angleMotor.setIdleMode(IdleMode.kCoast);
      driveMotor.setIdleMode(IdleMode.kCoast);
      
      driveEncoder = driveMotor.getEncoder();
      angleEncoder = angleMotor.getEncoder();
      
      driveEncoder.setPositionConversionFactor(kDriveEncoderRot2Meter);
      angleEncoder.setPositionConversionFactor(kAngleEncoderRot2Deg);
      
      resetRelEncoders();
    }
    
    public void resetRelEncoders() {
        driveEncoder.setPosition(0);
        angleEncoder.setPosition(getAngleDeg());
    }

    //conversion functions
    public double ticksToDegrees(double ticks) {
      return (ticks / kticksPerRevolution) * 360;
    }
    public double ticksToRotations(double ticks) {
      return ticks / kticksPerRevolution;
    }
    public double rotationsToTicks(double rot) {
      return rot * kticksPerRevolution;
    }
    public double r2dToTicks(Rotation2d r2d) {
      return (r2d.getDegrees() / 360) * kticksPerRevolution;
    }
    
    //encoder get functions
    public double getDriveEncoderPos() {
      return driveEncoder.getPosition();
    }

    public double getDriveEncoderVel() {
      return driveEncoder.getVelocity();
    }

    public double getAngleEncoderDeg() {
      return angleEncoder.getPosition();
    }
    
    public Rotation2d getAngleR2D() {
      return Rotation2d.fromDegrees(getAngleEncoderDeg()); 
    }
    public double getAngleDeg() {
      double angle = (getAbsoluteTicks() / kticksPerRevolution) * 360;
      return angle;
    }

    public double getRawAbsoluteTicks(){
      return magEncoder.get();
    }

    public double getAbsoluteTicks(){
      double magEncoderAbsValue = magEncoder.get();
      if (magEncoderAbsValue < 0)
      {
        magEncoderAbsValue = kticksPerRevolution + (magEncoder.get() % 1 ) * kticksPerRevolution;
      }
      else {
        magEncoderAbsValue = (magEncoder.get() % 1) * kticksPerRevolution;
      }
      double adjustedTicks = magEncoderAbsValue - r2dToTicks(offset);

      if (adjustedTicks > kticksPerRevolution) {
        adjustedTicks = adjustedTicks % kticksPerRevolution;
      }
      return adjustedTicks;
    }


    //:)
    /**
     * Set the speed + rotation of the swerve module from a SwerveModuleState object
     * @param desiredState - A SwerveModuleState representing the desired new state of the module
     */
    public void setDesiredState(SwerveModuleState desiredState) {

    Rotation2d currentAngleR2D = getAngleR2D();
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      angleMotor.set(0);
      driveMotor.set(0);
      return;
    }
    
    //Find the difference between our current rotational position and our new rotational position
    Rotation2d rotationDelta = desiredState.angle.minus(currentAngleR2D);

    //Find the new absolute position of the module based on the difference in degrees
    double deltaDeg = rotationDelta.getDegrees();

    if (Math.abs(deltaDeg) < 5) {
       angleMotorOutput = 0;
      }
    else {
      angleMotorOutput = angleController.calculate(getAngleEncoderDeg(), desiredState.angle.getDegrees());
    }

    SmartDashboard.putNumber("angle PID Output", angleMotorOutput);
  
    SmartDashboard.putNumber("state angle deg", desiredState.angle.getDegrees());
    SmartDashboard.putNumber("current angle deg", currentAngleR2D.getDegrees());
    SmartDashboard.putNumber("deltaDeg", deltaDeg);
    SmartDashboard.putNumber("rel angle", getAngleEncoderDeg());

    //comment out so robot doesn't explode
    angleMotor.set(angleMotorOutput);

    SmartDashboard.putNumber("drive output", desiredState.speedMetersPerSecond);
    SmartDashboard.putString("desiredState", desiredState.toString());
    //comment out so robot doesn't explode 
    driveMotor.set(desiredState.speedMetersPerSecond); // Motor smoky, check
    }

  }
}
//Lucia when ur mom