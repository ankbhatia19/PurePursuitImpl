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
public class Units {
    public static double feetToTicks(double feet) {
     
        /*
         Solve for x, where x is the amount of ticks
         x / meters = ticksPerRevolution / distancePerRevolution
         x = meters * (ticksPerRevolution / distancePerRevolution)
        */
        double distancePerRevolution = 2 * Math.PI * Constants.wheelRadius;

        return feet * (Constants.ticksPerRevolution / distancePerRevolution);
    }

    public static double ticksToFeet(double ticks) {
           /*
         Solve for x, where x is the amount of meters
         ticks / x = ticksPerRevolution / distancePerRevolution
         x = ticks * (distancePerRevolution / ticksPerRevolution)
        */
        double distancePerRevolution = 2 * Math.PI * Constants.wheelRadius;
        return ticks * (distancePerRevolution / Constants.ticksPerRevolution);
    }

    static final double TALON_TO_FPS_CONVERSION = .03067961572265625;           //( 1 rev/ 512 ticks) * (0.5pi ft/ 1 rev) * (10 [100ms] / 1 s)
    static final double FPS_TO_TALON_CONVERSION = 1 / TALON_TO_FPS_CONVERSION;

    public static double FPSToTalonNative(double fps) {
        return fps * FPS_TO_TALON_CONVERSION;
    }

    public static double TalonNativeToFPS(double nativeUnits) {
        return nativeUnits * .03067961572265625;
    }

    public static double TalonNativeToFeet(double nativeUnits) {
        return nativeUnits * .003067961;
    }

}
