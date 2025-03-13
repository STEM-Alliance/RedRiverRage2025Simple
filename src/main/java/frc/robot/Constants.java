// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /********************************************
     * Swerve and Controls
     *******************************************/
    // This controls the speed of the right to left slew rate. Large numbers mean
    // faster response
    static public double kVxSlewRateLimit = 6.0;
    static public double kVySlewRateLimit = 6.0;
    static public double kOmegaSlewRateLimit = 5.0;

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kOperatorButtonPanelPort = 2;
    public static final double kControllerDeadband = 0.2;
    public static final boolean kControllerScaling = true;

    // Max speeds
    static public double kMaxSpeed = 5.1; // 5.1 meters per second
    static public double kMaxAutonomousSpeed = 3; // this ensures that the max speed 2.5
    // (configured in pathplanne app settings) is never exceeded, should match the pathplanner max.
    static public double kMaxAngularSpeed = 4.0 * Math.PI; // 2 rotations per second 3.5
    static public double kMaxAutonomousAngularSpeed = kMaxAngularSpeed;
    static public double kMaxAngularAcceleration = Math.pow(2.0 * Math.PI, 2);
    static public double GeneralDeadband = 0.2;

    public static final double kMaxAlignmentSpeed = 1.0;
    public static final double kMaxAlignmentAngularSpeed = Math.PI / 1.0;

    // PID and Feedforward gains for the drive motors
    // NOTE: We are running the drive motors in open loop mode, using only the feedfoward.
    static public double kDriveKp = 0.0625;
    static public double kDriveKi = 0;
    static public double kDriveKd = 0;
    static public double kDriveKs = 0.27074 / 12.0;
    static public double kDriveKv = 2.7069 / 12.0;
    static public double kDriveKa = 0.5075 / 12.0;

    // PID and feedforward gains for the swerve motors
    static public double kSwerveKp = 0.375; // 0.0.5
    static public double kSwerveKi = 0; // 0.1
    static public double kSwerveKd = 0; // 0
    static public double kSwerveKs = 0.02;
    static public double kSwerveKv = 0.005;
    static public double kSwerveKa = 0.0;

    public static final PPHolonomicDriveController kPathplannerDriveController = new PPHolonomicDriveController(
        new PIDConstants(3.5, 0.0, 0.0), // Translation PID constants
        new PIDConstants(2.25, 0.0, 0.0) // Rotation PID constants
    );

    // Swerve Hardware
    static public double kWheelRadius = 0.04826;
    static public double kDriveGearReduction = 6.12;
    static public double kTurningGearReduction = 12.8;
    
    // FL, FR, BL, BR
    static public Translation2d[] kSwerveTranslations = new Translation2d[]{
        new Translation2d(0.276, 0.276),
        new Translation2d(0.276, -0.276),
        new Translation2d(-0.276, 0.276),
        new Translation2d(-0.276, -0.276)
    };

    // Module Index, Drive Motor Channel, Swerve Motor Channel, CANCoder Channel
    static public int[] kSwerveFLCanID = new int[]{0, 1, 2, 3};
    static public int[] kSwerveFRCanID = new int[]{1, 4, 5, 6};
    static public int[] kSwerveBLCanID = new int[]{2, 7, 8, 9};
    static public int[] kSwerveBRCanID = new int[]{3, 10, 11, 12};

    static public int kPigeon2CanID = 13;

    // NOTE: These seem to drift
    //static public double[] kZeroPosition = new double[]{0.963, 0.486, 0.070, 0.093};
    static public double[] kZeroPosition = new double[]{0.238, 0.2101, 0.704, 0.092};
    static public double kEncoderRes = 4096;

    /********************************************
     * Elevator and Shooter
     *******************************************/
    static public double kElevatorKp = 0.3;
    static public enum kElevatorSetpoints {
        L4      (57),
        L3      (32),
        L2      (18),
        L1      (5),
        CLIMB   (8),
        INTAKE  (0),
        IDLE    (0);
        
        private final double m_elevatorSetpoint;

        kElevatorSetpoints(double elevatorSetpoint) {
            m_elevatorSetpoint = elevatorSetpoint;
        }

        public double getAsDouble() {return m_elevatorSetpoint;}
    }

    static public double kShooterKp = 3.0;
    static public enum kShooterSetpoints {
        CLIMB      (0.82),
        INITIAL    (0.3638),
        L4_2       (0.7),
        L4_1       (0.7),
        L3_2       (0.72),
        L3_1       (0.72),
        L2_2       (0.72), 
        L2_1       (0.72), 
        L1_2       (0.72),
        L1_1       (0.72),
        INTAKE2    (0.35),
        INTAKE1    (0.72),
        IDLE       (0.534); 

        private final double m_shooterSetpoints;

        kShooterSetpoints(double shooterSetpoint) {
            m_shooterSetpoints = shooterSetpoint;
        }

        public double getAsDouble() {return m_shooterSetpoints;}
    }

    static public double kElevatorMaxVelocity = 50;
    static public double kElevatorMaxAcceleration = kElevatorMaxVelocity/0.5;
    static public double kShooterMaxVelocity = 2;
    static public double kShooterMaxAcceleration = kShooterMaxVelocity/0.15;

    static public int kRumbleTimer = 100;

    static public double kAlignXDistanceLeft = 0.41;
    static public double kAlignYDistanceLeft = -0.24;
    static public double kAlignXDistanceRight = 0.41;
    static public double kAlignYDistanceRight = 0.14;

    /********************************************
     * Motor Current Limits
     * P = VI
     * I = P / V
     *******************************************/
    static public int NeoLimit = 50;
    static public int Neo550Limit = 30;
    static public int BagMotorLimit = 30; // Max power is 149 W, 12.4 A
    static public int M775ProLimit = 15; // Max power 347 W, 28.9 A
    static public int CIMSLimit = 28; // Max power 337 W, 28.0 A
    // https://firstwiki.github.io/wiki/denso-window-motor
    static public int WindowLimit = 15; // This seems safe
}
