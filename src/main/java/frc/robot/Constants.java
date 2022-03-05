// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Color;

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

        //Max velocity in sensor units per 100ms
        public static final double MAX_VELOCITY = 17421;

        public static final int TIMEOUT_MS = 30;
        public static final int PID_LOOP_IDX = 0;
        public static final int SLOT_IDX = 0;
        // Reference: https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#calculating-velocity-feed-forward-gain-kf
        public static final double F = 0.8 * 1023 / 17421;
        public static final double P = 0.2;
        public static final double I = 0;
        public static final double D = 0;

        // Current limit for motors in amps
        public static final double CURRENT_LIMIT = 40;
        // Amps at which to trigger current limit
        public static final double TRIGGER_THRESHOLD_CURRENT = 45;
        // Trigger threshold must be attained for this amount of time to trigger current limit
        public static final double TRIGGER_THRESHOLD_TIME = 1;
    }

    public static final class TurretConstants {
        public static final int TURRET_TALON = 4;
        public static final int MAX_VELOCITY = 375;
        public static final int MAX_ACCELERATION = 375;

        // Degrees of Freedom
        public static double DEGREE = 1024;

        // PID CONTROL SPECIFIC CONSTANTS
        public static final int TIMEOUT_MS = 30;
        public static final int SLOT_IDX= 0;
        public static final int PIDLOOP_ID= 0;
        public static final double KF = .55;
        public static final double KP= .45;
        public static final double KI= 0;
        public static final double KD= 4.5;
        public static final double ALLOWABLE_ERROR= 1;
        public static final double I_ZONE= 0;
        public static final double MAX_INTEGRAL_ACCUMULATOR= 0;
        public static final double PEAK_OUTPUT= 12;
        public static final int SMOOTHING= 3;

        // LIMELIGHT ANGLE | Reference: https://docs.limelightvision.io/en/latest/cs_estimating_distance.html#using-area
        public static final double ANGLE_1 = (Math.PI/5); // 36 degrees
        public static final double HEIGHT_1 = 0.686; // Unit: meters | 27 inches
        public static final double HEIGHT_2 = 2.438; // Unit: meters | 8 feet
    }

    public static final class ShooterConstants {
        /*
            Refer to NEO spreadsheet for measurements
        */
        public static final int LEADER_SHOOTER_SPARK = 1;
        public static final int FOLLOWER_SHOOTER_SPARK = 2;

        public static final int MAX_RPM = 4410;
        public static final double MIN_SPEED = 0.7843;

        public static final int TIMEOUT_MS = 30;
        public static final int kPIDLoopIdx = 0;
        public static final int kP = 0;
        public static final int kI = 0;
        public static final int kD = 0;

        public static final double ACCEPTABLE_RPM_ERROR = 15;
    }

    public static final class ConveyorConstants {
        public static final int HORIZONTAL_CONVEYOR_SPARK = -1;
        public static final int VERTICAL_CONVEYOR_SPARK = -1;
        public static final int HORIZONTAL_BEAM_BREAK = 0;
        public static final int VERTICAL_BEAM_BREAK = 1;
    }
    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER = 0;
        public static final double DRIVER_CONTROLLER_DEADBAND = 0.05;
        public static final int TURRET_TALON = 7;
        public static final int MAX_VELOCITY = 214;
        public static final int MAX_ACCELERATION = 50;
    }

    public static final class IntakeConstants {
        public static final int LEADER_INTAKE_SPARK = 5;
        public static final int FOLLOWER_INTAKE_SPARK = 6;
        public static final int CTRE_PCM = 9;
        public static final int LEFT_PNEUMATIC_FORWARD = 5;
        public static final int LEFT_PNEUMATIC_REVERSE = 2;
        public static final int RIGHT_PNEUMATIC_FORWARD = 7;
        public static final int RIGHT_PNEUMATIC_REVERSE = 0;
    }

    public static final class ColorConstants {
        public static final Color BLUE_TARGET = new Color(0.143, 0.427, 0.429);
        public static final Color RED_TARGET = new Color(0.561, 0.232, 0.114);
    }
}
