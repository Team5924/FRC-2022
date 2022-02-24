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
        // Motor IDs
        public static final int LEFT_FRONT_TALON = 4;
        public static final int RIGHT_FRONT_TALON = 2;
        public static final int LEFT_BACK_TALON = 3;
        public static final int RIGHT_BACK_TALON = 1;

        //Max velocity in RPM
        public static final double MAX_VELOCITY = 17421;

        public static final int TIMEOUT_MS = 30;
        public static final int PID_LOOP_IDX = 0;
        public static final int SLOT_IDX = 0;
        public static final double F = 1023.0/20660.0;
        public static final double P = 0.1;
        public static final double I = 0.001;
        public static final double D = 5.0;

        // Current limit for motors in amps
        public static final double CURRENT_LIMIT = 40;
        // Amps at which to trigger current limit
        public static final double TRIGGER_THRESHOLD_CURRENT = 45;
        // Trigger threshold must be attained for this amount of time to trigger current limit
        public static final double TRIGGER_THRESHOLD_TIME = 1;
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DRIVER_CONTROLLER_DEADBAND = 0.05;
    }
}
