// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DrivetrainConstants {
        public static final int LEFT_MOTOR_PORT = 0;
        public static final int RIGHT_MOTOR_PORT = 1;
        public static final int LEFT_ENCODER_A = 4;
        public static final int LEFT_ENCODER_B = 5;
        public static final int RIGHT_ENCODER_A = 6;
        public static final int RIGHT_ENCODER_B = 7;

        public static final double COUNTS_PER_REVOLUTION = 1440.0;
        public static final double WHEEL_DIAMETER_INCH = 2.75591;
        public static final double WHEEL_DIAMETER_METERS = 0.7;
        public static final double INCHES_PER_PULSE = (Math.PI * WHEEL_DIAMETER_INCH) / COUNTS_PER_REVOLUTION;
        public static final double METERS_PER_PULSE = (Math.PI * WHEEL_DIAMETER_METERS) / COUNTS_PER_REVOLUTION;
    }

    public static final class AutoConstants {

        public static final double TRACKWIDTH_METERS = 0.142072613;

        public static final double S_VOLTS = 1.13;
        public static final double V_VOLT_SECONDS_PER_METER = 6.55;
        public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.0185;

        public static final double P_DRIVE_VEL = 0.00185;

        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;

        public static final double MAX_SPEED_METERS_PER_SECOND = 0.8;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.8;
        public static final double MAX_VOLTS = 9;


        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
            new DifferentialDriveKinematics(TRACKWIDTH_METERS);

        public final static SimpleMotorFeedforward DRIVE_FEED_FORWARD = new SimpleMotorFeedforward(
            S_VOLTS, V_VOLT_SECONDS_PER_METER, A_VOLT_SECONDS_SQUARED_PER_METER);

        public final static DifferentialDriveVoltageConstraint AUTO_VOLTAGE_CONSTRAINT = new DifferentialDriveVoltageConstraint(
            DRIVE_FEED_FORWARD, DRIVE_KINEMATICS, MAX_VOLTS);
    }
}
