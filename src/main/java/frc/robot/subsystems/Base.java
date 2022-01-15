// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import frc.robot.Gains;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Servo;
import com.revrobotics.CANSparkMax;

public class Base extends SubsystemBase {
   
  public static AHRS gyro = new AHRS(SPI.Port.kMXP);

  private SwerveDriveKinematics kinematics;

  private SwerveModuleMK3[] modules;

  private Servo linearActuator;

  public Base() {

    linearActuator = new Servo(KLinearActuator);
    linearActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);


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

  modules = new SwerveModuleMK3[] {
    
    new SwerveModuleMK3(new TalonFX(frontLeftDriveId), new TalonFX(frontLeftSteerId), new CANifier(frontLeftCANifierId), Rotation2d.fromDegrees(frontLeftOffset)), // Front Left
    new SwerveModuleMK3(new TalonFX(frontRightDriveId), new TalonFX(frontRightSteerId), new CANifier(frontRightCANifierId), Rotation2d.fromDegrees(frontRightOffset)), // Front Right
    new SwerveModuleMK3(new TalonFX(backLeftDriveId), new TalonFX(backLeftSteerId), new CANifier(backLeftCANifierId), Rotation2d.fromDegrees(backLeftOffset)), // Back Left
    new SwerveModuleMK3(new TalonFX(backRightDriveId), new TalonFX(backRightSteerId), new CANifier(backRightCANifierId), Rotation2d.fromDegrees(backRightOffset))  // Back Right

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
  SwerveDriveKinematics.normalizeWheelSpeeds(states, kMaxSpeed);
  // for (int i = 0; i < states.length; i++) {
  //   SwerveModuleMK3 module = modules[i];
  //   SwerveModuleState state = states[i];
  //   //below is a line to comment out from step 5
  //   module.setDesiredState(state);
  // }
  for (int i = 0; i < states.length; i = i + 2) {
    SwerveModuleMK3 module = modules[i];
    SwerveModuleState state = states[i];
    //below is a line to comment out from step 5
    module.setDesiredState(state);
  }

  // SwerveModuleMK3 module = modules[0];
  // SwerveModuleState state = states[0];
  // module.setDesiredState(state);


    //SmartDashboard.putNumber("gyro Angle", gyro.getAngle());
  }

  public void linearActuatorMove(double amount){
    System.out.println("================> amount " + amount);
    linearActuator.setSpeed(amount);
  }

  public void resetGyro() {
    gyro.reset(); //recalibrates gyro offset
  }

  //DOESNT WORK!!
  public void resetAllAngleMotors(){
    // for (int i = 0; i < modules.length; i = i++) {
    //   SwerveModuleMK3 module = modules[i];
    //   module.resetAngleMotor();
    // }
    // for (int i = 0; i < modules.length; i = i+ 2) {
      SwerveModuleMK3 module = modules[2];
      module.resetAngleMotor();
    // }
  }

  public void zeroEncoder() {
    modules[0].zeroEncoders();
    modules[1].zeroEncoders();
    modules[2].zeroEncoders();
    modules[3].zeroEncoders();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Front Raw Angle", modules[0].getRawAngle());
    SmartDashboard.putNumber("Right Front Raw Angle", modules[1].getRawAngle());
    SmartDashboard.putNumber("Left Back Raw Angle", modules[2].getRawAngle());
    SmartDashboard.putNumber("Right Back Raw Angle", modules[3].getRawAngle());
    SmartDashboard.putNumber("Desired Ticks", modules[3].getDesiredTicks());
    SmartDashboard.putNumber("Closed Loop Target", modules[3].getSelectedSensonPosition());

    // setModuleGains(SmartDashboard.getNumber("Base kP", 0.0), SmartDashboard.getNumber("Base kI", 0.0), SmartDashboard.getNumber("Base kD", 0.0));

    SmartDashboard.putNumber("Current Tick", modules[3].getCurrentTicks());
    //SmartDashboard.putNumber("SetPoint", modules[3].getSetpoint());
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

  public class SwerveModuleMK3 {

    private  double maxDeltaTicks = 980.0;

    private  final Gains kDriveGains = new Gains(15, 0.01, 0.1, 0.2, 0, 1.0);

    private final Gains kAngleGains = new Gains(0.6, 0.0, 0.25, 0.0, 0, 1.0);

    // CANCoder has 4096 ticks/rotation
    private double kEncoderTicksPerRotation = 4096;

    private double desiredTicks;

    private TalonFX driveMotor;
    private TalonFX angleMotor;
    private CANifier canifier;
    private Rotation2d offset;
    //private Boolean invert;

    SwerveModuleMK3(TalonFX driveMotor, TalonFX angleMotor, CANifier canifier, Rotation2d offset) {
      this.driveMotor = driveMotor;
      this.angleMotor = angleMotor;
      this.canifier = canifier;
      this.offset = offset;
      //this.invert = invert;

      //angleMotor.configAllowableClosedloopError(0, 0, 0);

      /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
      // angleMotor.config_kP(0, kAngleGains.kP, 0);
      // angleMotor.config_kI(0, kAngleGains.kI, 0);
      // angleMotor.config_kD(0, kAngleGains.kD, 0);
      
      angleMotor.setNeutralMode(NeutralMode.Brake); //not needed but nice to keep the robot stopped when you want it stopped

      angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

      //driveMotor.configAllowableClosedloopError(0, 0, 0);

      /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
      // driveMotor.config_kF(0, kDriveGains.kF, 0);
      // driveMotor.config_kP(0, kDriveGains.kP, 0);
      // driveMotor.config_kI(0, kDriveGains.kI, 0);
      // driveMotor.config_kD(0, kDriveGains.kD, 0);

      //driveMotor.setInverted(invert);
      driveMotor.setNeutralMode(NeutralMode.Brake);
    }


    /**
     * Gets the relative rotational position of the module
     * @return The relative rotational position of the angle motor in degrees
     */
    public Rotation2d getAngle() {
      double deg = (canifier.getQuadraturePosition() % ticksPerRevolution) * 360 / ticksPerRevolution;
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

      double deg = (canifier.getQuadraturePosition() % ticksPerRevolution) * 360 / ticksPerRevolution;
      if (deg < 0){
        deg = deg + 360;
      }

      //deg *= 10;
      //deg = (int) deg;
      //deg /= 10;

      return deg; //include angle offset
    }

    public double getCurrentTicks() {
      return canifier.getQuadraturePosition();
    }

    public double getDesiredTicks() {
      return desiredTicks;
    }

    public double getSelectedSensonPosition() {
      return angleMotor.getSelectedSensorPosition();
    }

    public void zeroEncoders() {
      angleMotor.setSelectedSensorPosition(0, 0, 0);
      canifier.setQuadraturePosition(0, 0);
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
      double currentTicks = canifier.getQuadraturePosition();

      desiredTicks = deltaTicks;

      if (desiredTicks > maxDeltaTicks) {
        desiredTicks = maxDeltaTicks;
      } else if (desiredTicks < -maxDeltaTicks) {
        desiredTicks = -maxDeltaTicks;
      }

      //below is a line to comment out from step 5
      angleMotor.set(TalonFXControlMode.Position, desiredTicks);

      double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond)/2;

      //below is a line to comment out from step 5
      driveMotor.set(TalonFXControlMode.PercentOutput, feetPerSecond / kMaxSpeed);
    }

    public void resetAngleMotor(){
      angleMotor.set(TalonFXControlMode.Position, 0);
    }

    public void setAnglePIDGains(double kP, double kI, double kD){
      //angleMotor.config_kP(0, kP, 0);
      //angleMotor.config_kI(0, kI, 0);
      //angleMotor.config_kD(0, kD, 0);
    }
  }
    
}