// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int LEFT_FRONT_TALON = 2;
        public static final int RIGHT_FRONT_TALON = 0;
        public static final int LEFT_BACK_TALON = 3;
        public static final int RIGHT_BACK_TALON = 1;
    }

    public static final class TurretConstants {
        public static final int TURRET_TALON = 4;
        public static final int MAX_VELOCITY = 375;
        public static final int MAX_ACCELERATION = 187;
        
        
        //PID CONTROL SPECIFIC CONSTANTS
        public static final int kTimeoutMs = 30;
        public static final int kSlotIdx= 0;
        public static final int kPIDLoopIdx= 0;
        public static double kF = .55;
        public static final double kP= .35;
        public static final double kI= 0;    
        public static final double kD= 0;
        public static final double AllowableError= 1;
        public static final double IZone= 0;
        public static final double MaxIntegralAccumulator= 0;
        public static final double PeakOutput= 12;
        public static final int Smoothing= 1;
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}
