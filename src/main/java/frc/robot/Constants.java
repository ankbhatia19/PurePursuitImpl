/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Constants {

    //ALL UNITS ARE IN FEET
    public static double wheelRadius = 2.5 / 12;
    public static double maxVelocity = 1.7;
    public static double maxAcceleration = 2;
    public static double wheelBaseWidth = (double)22 / 12;
    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0;

    public static double kV = 1; //needs tuning
    public static int ticksPerRevolution = 4069;

    public static int lookaheadDistance = 3;
}
