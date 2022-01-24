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
    public static final int frontLeftDriveId = 3; 
    public static final int frontLeftMagEncoderId = 1; 
    public static final int frontLeftSteerId = 4;
    public static final int frontRightDriveId = 1; 
    public static final int frontRightMagEncoderId = 0; 
    public static final int frontRightSteerId = 2; 
    public static final int backLeftDriveId = 5; 
    public static final int backLeftMagEncoderId = 2; 
    public static final int backLeftSteerId = 6;
    public static final int backRightDriveId = 7; 
    public static final int backRightMagEncoderId = 3; 
    public static final int backRightSteerId = 8;

    //Storage
    // public static final int KStorageSpark = 18;

    //Shooter
    // public static final int KShooterSpark = 14;

    //Intake
    // public static final int KIntakeSpark = 9;

    //Hang
    // public static final int KLeftHangMotor = 15;
    // public static final int KRightHangMotor = 17;
    // public static final int KMiddleHangMotor = 19;

// Default PWM Values

    //Intake
    public static final double KIntakePWM = 1;

//Base Constants
    public static final double kMaxSpeed = Units.feetToMeters(13.6) / 100; // 20 feet per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    public static final double kticksPerRevolution = 4096;
    public static double feildCalibration = 0;
    public static double frontLeftOffset = 0;
    public static double frontRightOffset = 0;
    public static double backLeftOffset = 0;
    public static double backRightOffset = 0;

    public static final int KLinearActuator = 5;
}
