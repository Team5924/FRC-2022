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

        // Max velocity that the drivetrain can do in sensor units per 100ms
        public static final double MAX_VELOCITY = 21776.25;
        // Percentage to limit drivetrain to (of above number)
        public static final double PERCENT_MAX_VELOCITY_LIMIT = 0.7;

        public static final int TIMEOUT_MS = 30;
        public static final int PID_LOOP_IDX = 0;
        public static final int SLOT_IDX = 0;
        public static final double F = 0.8 * 1023 / (MAX_VELOCITY * PERCENT_MAX_VELOCITY_LIMIT);
        public static final double P = 0.18;
        public static final double I = 0;
        public static final double D = 1.8;

        // Current limit for motors in amps
        public static final double CURRENT_LIMIT = 40;
        // Amps at which to trigger current limit
        public static final double TRIGGER_THRESHOLD_CURRENT = 45;
        // Trigger threshold must be attained for this amount of time to trigger current limit
        public static final double TRIGGER_THRESHOLD_TIME = 1;
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DRIVER_CONTROLLER_DEADBAND = 0.08;
    }

    public static final class TurretConstants {
        public static final int TURRET_SPARK = 7;

        // LIMELIGHT ANGLE | Reference: https://docs.limelightvision.io/en/latest/cs_estimating_distance.html#using-area
        public static final double ANGLE_1 = (Math.PI/5); // 36 degrees
        public static final double HEIGHT_1 = 0.686; // Unit: meters | 27 inches
        public static final double HEIGHT_2 = 2.438; // Unit: meters | 8 feet

        public static final double P = 0.0001;
        public static final double I = 0;
        public static final double D = 0.001;
        public static final double I_ZONE = 0;
        public static final double FF = 0;

        public static final int ACCEPTABLE_ERROR = 2;
        public static final double TURRET_GEARBOX_RATIO = 297.5;
    }
}
