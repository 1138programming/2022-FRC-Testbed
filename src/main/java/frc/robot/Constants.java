// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Motors IDs
    //Base
    // public static final int frontLeftDriveId = 1; 
    public static final int frontLeftCANifierId = 2; 
    // public static final int frontLeftSteerId = 8;
    public static final int frontRightDriveId = 7; 
    public static final int frontRightCANifierId = 0; 
    public static final int frontRightSteerId = 5; 
    // public static final int backLeftDriveId = 6; 
    public static final int backLeftCANifierId = 10; 
    // public static final int backLeftSteerId = 3;
    // public static final int backRightDriveId = 9; 
    public static final int backRightCANifierId = 12; 
    // public static final int backRightSteerId = 4;

    //Storage
    public static final int KStorageSpark = 9;

    //Shooter
    public static final int KLeftShooterMotor = 11;
    public static final int KRightShooterMotor = 12;

    //Intake
    public static final int KIntakeMotor = 10;

    //Hang
    public static final int KLeftHangMotor = 15;
    public static final int KRightHangMotor = 17;
    public static final int KMiddleHangMotor = 19;

    public static final int KLeftLinearServo = 1;
    public static final int KMiddleLinearServo = 2;
    public static final int KRightLinearServo = 3;

// Default PWM Values

    //Intake
    public static final double KIntakePWM = 1.0;

//Base Constants
    public static final double kMaxSpeed = Units.feetToMeters(13.6) / 100; // 20 feet per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    public static final double ticksPerRevolution = 4096;
    public static double feildCalibration = 0;
    public static double frontLeftOffset = 0;
    public static double frontRightOffset = 0;
    public static double backLeftOffset = 0;
    public static double backRightOffset = 0;
    public static double kDriveP = 1;
    public static double kDriveI = 0;
    public static double kDriveD = 0;
    public static double kAngleP = 1;
    public static double kAngleI = 0;
    public static double kAngleD = 0;

    public static final int KLinearActuator = 5;
}
