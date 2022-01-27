// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class kOI {
        public static final String TRAJECTORY_NAME = "path";
    }

    public static final class kDrive {
        
        // motor constants
        public static final int FRONT_LEFT_ID = 1;
        public static final int FRONT_RIGHT_ID = 3;
        public static final int BACK_LEFT_ID = 2;
        public static final int BACK_RIGHT_ID = 4;

        // pidf constants - find via characterization
        /*
        public static final double ksVolts = 0.48046;
        public static final double kvVoltSecondsPerMeter = 1.7496;
        public static final double kaVoltSecondsSquaredPerMeter = 0.17128;
        public static final double kPDriveVel = 2.1474; */

        // char values for garage
        public static final double ksVolts = 0.54849;
        public static final double kvVoltSecondsPerMeter = 1.6912;
        public static final double kaVoltSecondsSquaredPerMeter = 0.21572;
        public static final double kPDriveVel = 0.00005;
        public static final double kIDrive = 0;
        public static final double kDDrive = 0;

        // public static final double kPDriveVel = 0;
        public static final double MAX_VOLTAGE = 5;

        // odometry constants - drivetrain measurements
        public static final double TRACK_WIDTH_METERS = 0.69; // width between sides of dt
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = 
            new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

        // path-following constants
        public static final double MAX_SPEED_METERS_PER_SECOND = 3; // set to somewhat below free speed
                                                                 // could increase this to go faster 
                                                                 // theoretically
        public static final double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 3; // doesn't really matter

        // ticks to meters conversion factor for falcon 500
        // (total ticks) * (motor rotations/tick) * (wheel rotations/motor rotations) * (meters/wheel rotations)
        public static final double TICKS_TO_METERS = (1.0/2048.0) * (1.0/7.31) * (0.4788);

        // ramsete parameters
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;
    }

}
