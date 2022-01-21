// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
   
  public static AHRS gyro = new AHRS(SPI.Port.kMXP);

  private SwerveDriveKinematics kinematics;

  private ServeX[] modules;

  private double KAbsoluteResetPoint = 69;

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

  modules = new ServeX[] {
    
    // new ServeX(new CANSparkMax(frontLeftDriveId, MotorType.kBrushless), new CANSparkMax(frontLeftSteerId, MotorType.kBrushless), new DutyCycleEncoder(frontLeftCANifierId), Rotation2d.fromDegrees(frontLeftOffset)), // Front Left
    new ServeX(new CANSparkMax(frontRightDriveId, MotorType.kBrushless), new CANSparkMax(frontRightSteerId, MotorType.kBrushless), new DutyCycleEncoder(frontRightCANifierId), Rotation2d.fromDegrees(frontRightOffset)), // Front Right
    // new ServeX(new CANSparkMax(backLeftDriveId, MotorType.kBrushless), new CANSparkMax(backLeftSteerId, MotorType.kBrushless), new DutyCycleEncoder(backLeftCANifierId), Rotation2d.fromDegrees(backLeftOffset)), // Back Left
    // new ServeX(new CANSparkMax(backRightDriveId, MotorType.kBrushless), new CANSparkMax(backRightSteerId, MotorType.kBrushless), new DutyCycleEncoder(backRightCANifierId), Rotation2d.fromDegrees(backRightOffset))  // Back Right
  };

  SmartDashboard.putNumber("Base kP", 0.0);
  SmartDashboard.putNumber("Base kI", 0.0);
  SmartDashboard.putNumber("Base kD", 0.0);
  
  gyro.reset(); 
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param calibrateGyro button to recalibrate the gyro offset
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
    //if(calibrateGyro){
      //gyro.reset(); //recalibrates gyro offset
    //}

  SwerveModuleState[] states =
    kinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-gyro.getAngle()))
        : new ChassisSpeeds(xSpeed, ySpeed, rot));
  SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeed);
  // for (int i = 0; i < states.length; i++) {
  //   ServeX module = modules[i];
  //   SwerveModuleState state = states[i];
  //   //below is a line to comment out from step 5
  //   module.setDesiredState(state);
  // }
  // for (int i = 0; i < states.length; i = i++) {
  //   ServeX module = modules[i];
  //   SwerveModuleState state = states[i];
  //   //below is a line to comment out from step 5
  //   module.setDesiredState(state);
  // }

  ServeX module = modules[0];
  SwerveModuleState state = states[0];
  module.setDesiredState(state);


    //SmartDashboard.putNumber("gyro Angle", gyro.getAngle());
  }

  public void resetGyro() {
    gyro.reset(); //recalibrates gyro offset
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Rght Front Absolute Angle", modules[0].getAbsoluteTicks());

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

  class ServeX {

    private  double maxDeltaTicks = 980.0;

    private  final Gains kDriveGains = new Gains(15, 0.01, 0.1, 0.2, 0, 1.0);

    private final Gains kAngleGains = new Gains(0.6, 0.0, 0.25, 0.0, 0, 1.0);
    
    private PIDController driveController, angleController;


    // CANCoder has 4096 ticks/rotation
    private double kEncoderTicksPerRotation = 4096;

    private double desiredTicks;

    private CANSparkMax driveMotor;
    private CANSparkMax angleMotor;
    private DutyCycleEncoder magEncoder;
    private Rotation2d offset;
    private RelativeEncoder angleEncoder, driveEncoder;
    private double[] pulseWidthAndPeriod = new double[]{1, 1/244}; //pulse width found in mag encoder manual pdf, period is 1/frequency (also found in pdf)

    ServeX(CANSparkMax driveMotor, CANSparkMax angleMotor, DutyCycleEncoder magEncoder, Rotation2d offset) {
      this.driveMotor = driveMotor;
      this.angleMotor = angleMotor;
      this.magEncoder = magEncoder;
      this.offset = offset;

      angleEncoder = angleMotor.getEncoder(); // check ticks per revolution with neos
      driveEncoder = driveMotor.getEncoder(); // check ticks per revolution with neos

      //this.invert = invert;
      //PIDControllers:
      driveController = new PIDController(kDriveP, kDriveI, kDriveD);
      angleController = new PIDController(kAngleP, kAngleI, kAngleD);


      //angleMotor.configAllowableClosedloopError(0, 0, 0);

      /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
      // angleMotor.config_kP(0, kAngleGains.kP, 0);
      // angleMotor.config_kI(0, kAngleGains.kI, 0);
      // angleMotor.config_kD(0, kAngleGains.kD, 0);
      
      angleMotor.setIdleMode(IdleMode.kBrake); //not needed but nice to keep the robot stopped when you want it stopped

      //driveMotor.configAllowableClosedloopError(0, 0, 0);

      /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
      // driveMotor.config_kF(0, kDriveGains.kF, 0);
      // driveMotor.config_kP(0, kDriveGains.kP, 0);
      // driveMotor.config_kI(0, kDriveGains.kI, 0);
      // driveMotor.config_kD(0, kDriveGains.kD, 0);

      //driveMotor.setInverted(invert);
      driveMotor.setIdleMode(IdleMode.kBrake);
    }

    public void moveToPos(double pos){
      
    }

    /**
     * Gets the relative rotational position of the module
     * @return The relative rotational position of the angle motor in degrees
     */
    public Rotation2d getAngle() {
      double deg = (angleEncoder.getPosition() % 1) * 360; //getPosition returns rotations (e.g. 5.25 rotations), so modulo 1 gets remainder (0.25), * 360 to get degrees
      //double deg = (canifier.getQuadraturePosition() % ticksPerRevolution) * 360 / ticksPerRevolution;
      //double deg = canifier.getQuadraturePosition() * 360.0 / 4096.0;
      if (deg < 0){
        deg = deg +360;
      }
      //deg *= 10;
      //deg = (int) deg;
      //deg /= 10;
      
      return Rotation2d.fromDegrees(deg); //include angle offset
    }
    public double getRawAngle() {
      //double deg = canifier.getQuadraturePosition() * 360.0 / 4096.0;

      double deg = (angleEncoder.getPosition() % 1) * 360;
    //   double deg = (canifier.getQuadraturePosition() % ticksPerRevolution) * 360 / ticksPerRevolution;
      if (deg < 0){
        deg = deg + 360;
      }

      //deg *= 10;
      //deg = (int) deg;
      //deg /= 10;

      return deg; //include angle offset
    }

    public double getCurrentTicks() {
      return angleEncoder.getPosition() * ticksPerRevolution;
    //   return canifier.getQuadraturePosition();
    }

    public double getDesiredTicks() {
      return desiredTicks;
    }

    public double getAbsoluteTicks(){
      return (magEncoder.get() % 1) * ticksPerRevolution;
    }

    public void resetEncoders() {
      angleEncoder.setPosition(KAbsoluteResetPoint - magEncoder.get()); //all units in rotations, Need to get mag encoder PWN output from Rio for absolute reading
    //   canifier.setQuadraturePosition(KAbsoluteResetPoint - canifier.getPWMInput(magEncoder, pulseWidthAndPeriod); //Need to get PWN output from canifier into Rio for absolute reading
    }
    //:)
    /**
     * Set the speed + rotation of the swerve module from a SwerveModuleState object
     * @param desiredState - A SwerveModuleState representing the desired new state of the module
     */
    public void setDesiredState(SwerveModuleState desiredState) {

      Rotation2d getAngle = getAngle();
      SwerveModuleState state = SwerveModuleState.optimize(desiredState, getAngle);
      
      // Find the difference between our current rotational position + our new rotational position
      Rotation2d rotationDelta = state.angle.minus(getAngle);
      
      // Find the new absolute position of the module based on the difference in rotation
      double deltaTicks = (rotationDelta.getDegrees() / 360) * kEncoderTicksPerRotation;
      // Convert the CANCoder from it's position reading back to ticks
      // double currentTicks = canifier.getQuadraturePosition();
      
      desiredTicks = deltaTicks;

      if (desiredTicks > maxDeltaTicks) {
        desiredTicks = maxDeltaTicks;
      } else if (desiredTicks < -maxDeltaTicks) {
        desiredTicks = -maxDeltaTicks;
      }

      // angleMotor.set(angleController.calculate(angleEncoder.getPosition(), desiredTicks));

      double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond)/2;

      // driveMotor.set(driveController.calculate(driveEncoder.getVelocity(), feetPerSecond / kMaxSpeed));
    }

    public void setAnglePIDGains(double kP, double kI, double kD){
      //angleMotor.config_kP(0, kP, 0);
      //angleMotor.config_kI(0, kI, 0);
      //angleMotor.config_kD(0, kD, 0);
    }
  }
    
}