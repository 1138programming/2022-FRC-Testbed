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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.AngleStatistics;

public class NeoBase extends SubsystemBase {
   
  public static AHRS gyro = new AHRS(SPI.Port.kMXP); //not on the bot yet

  private SwerveDriveKinematics kinematics;

  private SwerveX[] modules;

  // private double[] KAbsoluteResetPoints = {3192, 3823, 3220, 2280};
  private double frontLeftOffset = ((kticksPerRevolution - 3192) / kticksPerRevolution) * 360 ;
  private double frontRightOffset = ((kticksPerRevolution - 3823) / kticksPerRevolution) * 360;
  private double backLeftOffset = ((kticksPerRevolution - 3220) / kticksPerRevolution) * 360;
  private double backRightOffset = ((kticksPerRevolution - 2280) / kticksPerRevolution) * 360;

  public NeoBase() {

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

  modules = new SwerveX[] {
    new SwerveX(new CANSparkMax(frontLeftDriveId, MotorType.kBrushless), new CANSparkMax(frontLeftSteerId, MotorType.kBrushless), new DutyCycleEncoder(frontLeftMagEncoderId), Rotation2d.fromDegrees(frontLeftOffset)), // Front Left
    new SwerveX(new CANSparkMax(frontRightDriveId, MotorType.kBrushless), new CANSparkMax(frontRightSteerId, MotorType.kBrushless), new DutyCycleEncoder(frontRightMagEncoderId), Rotation2d.fromDegrees(frontRightOffset)), // Front Right
    new SwerveX(new CANSparkMax(backLeftDriveId, MotorType.kBrushless), new CANSparkMax(backLeftSteerId, MotorType.kBrushless), new DutyCycleEncoder(backLeftMagEncoderId), Rotation2d.fromDegrees(backLeftOffset)), // Back Left
    new SwerveX(new CANSparkMax(backRightDriveId, MotorType.kBrushless), new CANSparkMax(backRightSteerId, MotorType.kBrushless), new DutyCycleEncoder(backRightMagEncoderId), Rotation2d.fromDegrees(backRightOffset))  // Back Right
  };

  SmartDashboard.putNumber("Base Angle kP", 0.0);
  SmartDashboard.putNumber("Base Angle kI", 0.0);
  SmartDashboard.putNumber("Base Angle kD", 0.0);
  
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
  SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeed);
  // for (int i = 0; i < states.length; i++) {
  //   SwerveX module = modules[i];
  //   SwerveModuleState state = states[i];
  //   module.setDesiredState(state);
  // }
  // for (int i = 0; i < states.length; i = i++) {
  //   SwerveX module = modules[i];
  //   SwerveModuleState state = states[i];
  //   module.setDesiredState(state);
  // }

  SwerveX module = modules[0];
  SwerveModuleState state = states[0];
  module.setDesiredState(state);
  }

  // public void resetGyro() {
  //   gyro.reset(); //recalibrates gyro offset
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Front Absolute Angle", modules[0].getAbsoluteTicks());
    // SmartDashboard.putNumber("Rght Front Absolute Angle", modules[1].getAbsoluteTicks());
    // SmartDashboard.putNumber("Left Back Absolute Angle", modules[2].getAbsoluteTicks());
    // SmartDashboard.putNumber("Rght Back Absolute Angle", modules[3].getAbsoluteTicks());
    SmartDashboard.putNumber("Left Front Raw Angle", modules[0].getRawAbsoluteTicks());
    // SmartDashboard.putNumber("Right Front Rel Angle", modules[1].getAngleTicks());
    // SmartDashboard.putNumber("Left Back Rel Angle", modules[2].getAngleTicks());
    // SmartDashboard.putNumber("Right Back Rel Angle", modules[3].getAngleTicks());

    // setModuleGains(SmartDashboard.getNumber("Base kP", 0.0), SmartDashboard.getNumber("Base kI", 0.0), SmartDashboard.getNumber("Base kD", 0.0));
    // This method will be called once per scheduler run
  }

  public void setModuleGains(double kP, double kI, double kD){
    // modules[1].setAnglePIDGains(kP, kI, kD);
    // modules[2].setAnglePIDGains(kP, kI, kD);
    // modules[3].setAnglePIDGains(kP, kI, kD);
    // modules[4].setAnglePIDGains(kP, kI, kD);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  class SwerveX {

    private  double maxDeltaTicks = 980.0;

    private  final Gains kDriveGains = new Gains(15, 0.01, 0.1, 0.2, 0, 1.0);

    private final Gains kAngleGains = new Gains(0.6, 0.0, 0.25, 0.0, 0, 1.0);

    private double kEncoderTicksPerRotation = 4096;
    private double kMotorShaftToWheelRatio = 72/7;

    private double desiredTicks;

    private CANSparkMax driveMotor;
    private CANSparkMax angleMotor;
    private DutyCycleEncoder magEncoder;
    private PIDController driveController, angleController;
    private Rotation2d offset;
    private RelativeEncoder angleEncoder, driveEncoder;
    private double[] pulseWidthAndPeriod = new double[]{1, 1/244}; //pulse width found in mag encoder manual pdf, period is 1/frequency (also found in pdf)

    SwerveX(CANSparkMax driveMotor, CANSparkMax angleMotor, DutyCycleEncoder magEncoder, Rotation2d offset) {
      this.driveMotor = driveMotor;
      this.angleMotor = angleMotor;
      this.magEncoder = magEncoder;
      this.offset = offset;

      //PIDControllers:
      driveController = new PIDController(kDriveP, kDriveI, kDriveD);
      angleController = new PIDController(kAngleP, kAngleI, kAngleD);

      angleMotor.setIdleMode(IdleMode.kCoast); //not needed but nice to keep the robot stopped when you want it stopped
      driveMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Gets the relative rotational position of the module
     * @return The relative rotational position of the angle motor in degrees
     */
    public Rotation2d getAngleR2D() {
      double deg = (getAbsoluteTicks() / kticksPerRevolution) * 360; 
      return Rotation2d.fromDegrees(deg).plus(offset); 
    }
    public double getAngleDeg() {
      double angle = (getAbsoluteTicks() / kticksPerRevolution) * 360 + offset.getDegrees();
      return angle;
    }

    public double getRawAbsoluteTicks(){
      double magEncoderAbsValue = magEncoder.get();
      if (magEncoderAbsValue < 0)
      {
        magEncoderAbsValue = kticksPerRevolution + (magEncoder.get() % 1 ) * kticksPerRevolution;
      }
      else {
        magEncoderAbsValue = (magEncoder.get() % 1) * kticksPerRevolution;
      }
      return magEncoderAbsValue;
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
      return magEncoderAbsValue + r2dToTicks(offset);
    }
    
    public double getDesiredTicks() {
      return desiredTicks;
    }

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

    //:)
    /**
     * Set the speed + rotation of the swerve module from a SwerveModuleState object
     * @param desiredState - A SwerveModuleState representing the desired new state of the module
     */
    public void setDesiredState(SwerveModuleState desiredState) {

    Rotation2d angleR2D = getAngleR2D();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, angleR2D);
    
    // Find the difference between our current rotational position + our new rotational position
    Rotation2d rotationDelta = state.angle.minus(angleR2D);
    
    // Find the new absolute position of the module based on the difference in rotation
    double deltaTicks = (rotationDelta.getDegrees() / 360) * kticksPerRevolution;
  
    if (deltaTicks > maxDeltaTicks) {
      deltaTicks = maxDeltaTicks;
    } else if (deltaTicks < -maxDeltaTicks) {
      deltaTicks = -maxDeltaTicks;
    }

    //commented out so robot doesn't explode
    // angleMotor.set(angleController.calculate(deltaTicks, 0)); //close the gap, work pls?

    double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond)/2;

    //commented out so robot doesn't explode
    // driveMotor.set(driveController.calculate(driveEncoder.getVelocity(), feetPerSecond / kMaxSpeed));
    }

    public void setAnglePIDGains(double kP, double kI, double kD){
      //angleMotor.config_kP(0, kP, 0);
      //angleMotor.config_kI(0, kI, 0);
      //angleMotor.config_kD(0, kD, 0);
    }
  }
}