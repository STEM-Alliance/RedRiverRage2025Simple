// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.utils.DataLogHelpers;
import frc.robot.Constants;
import frc.robot.misc.SwerveModule;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class DrivetrainSubsystem extends SubsystemBase {
  // https://pathplanner.dev/pplib-build-an-auto.html#create-a-sendablechooser-with-all-autos-in-project

  // Locations for the swerve drive modules relative to the robot center.
  Translation2d m_frontLeftLocation = Constants.kSwerveTranslations[0];
  Translation2d m_frontRightLocation = Constants.kSwerveTranslations[1];
  Translation2d m_backLeftLocation = Constants.kSwerveTranslations[2];
  Translation2d m_backRightLocation = Constants.kSwerveTranslations[3];

  private final int[] frontLeftChannels = Constants.kSwerveFLCanID;
  private final int[] frontRightChannels = Constants.kSwerveFRCanID;
  private final int[] backLeftChannels = Constants.kSwerveBLCanID;
  private final int[] backRightChannels = Constants.kSwerveBRCanID;

  private final SwerveModule m_frontLeft = new SwerveModule(
    frontLeftChannels[0], frontLeftChannels[1], frontLeftChannels[2], frontLeftChannels[3]);

  private final SwerveModule m_frontRight = new SwerveModule(
    frontRightChannels[0], frontRightChannels[1], frontRightChannels[2], frontRightChannels[3]);

  private final SwerveModule m_backLeft = new SwerveModule(
    backLeftChannels[0], backLeftChannels[1], backLeftChannels[2], backLeftChannels[3]);

  private final SwerveModule m_backRight = new SwerveModule(
    backRightChannels[0], backRightChannels[1], backRightChannels[2], backRightChannels[3]);
      
  private final SwerveModule m_modules[] = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};
  private final Pigeon2 m_pigeon2 = new Pigeon2(Constants.kPigeon2CanID);

  // Creating my kinematics object using the module locations
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  boolean m_turbo = false;

  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    m_kinematics,
    m_pigeon2.getRotation2d(),
    getModulePositions(),
    new Pose2d()
  );

  private final Field2d m_field = new Field2d();
  double m_desiredAngle = 0;

  /** Creates a new DriveSubSystem. */
  public DrivetrainSubsystem() {
    m_pigeon2.reset();

    // Set deviations for the pose estimator vision measurements, rotation is positive infinity since
    // the gyro will give us more accurate results than the vision system. We should also scale this by the
    // distance of the tag detected, longer distances will be less accurate so will have less of an effect

    // Actual deviations are 0.5, over time odometry becomes less accurate?
    // Deviaton for rotation should be Double.POSITIVE_INFINITY
    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, Double.POSITIVE_INFINITY));

    RobotConfig config;

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();

      // TODO find a better solution
      config = new RobotConfig(null, null, null, null, null);
    }

    AutoBuilder.configure(
              this::getPose, // Robot pose supplier
              this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              (speeds, feedforwards) -> driveRobotSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
              new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                      new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                      new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
              ),
              config, // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
      );

    // TODO: Fixme
    // https://github.com/mjansen4857/pathplanner/tree/main/examples
    // Configure AutoBuilder
    // AutoBuilder.configureHolonomic(
    //   this::getPose,
    //   this::resetPose,
    //   this::getChassisSpeeds,
    //   this::driveRobotSpeeds,
    //   Constants.kPathFollowerConfig,

    //   () -> {
    //       // Boolean supplier that controls when the path will be mirrored for the red alliance
    //       // This will flip the path being followed to the red side of the field.
    //       // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //       var alliance = DriverStation.getAlliance();
    //       if (alliance.isPresent()) {
    //           return alliance.get() == DriverStation.Alliance.Red;
    //       }
    //       return false;
    //   },

    //   this);

    //       // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
  }

  public void periodic() {
    updateOdometry();
    m_field.setRobotPose(getPose());

    DataLogHelpers.logDouble(m_pigeon2.getYaw().getValue().in(Units.Degrees), "Pigeon2Yaw");
    DataLogHelpers.logDouble(getPose().getX(), "RobotPoseX");
    DataLogHelpers.logDouble(getPose().getY(), "RobotPoseY");
    DataLogHelpers.logDouble(getPose().getRotation().getDegrees(), "RobotPoseDeg");
    DataLogHelpers.logDouble(m_poseEstimator.getEstimatedPosition().getX(), "RobotPoseX2");
    DataLogHelpers.logDouble(m_poseEstimator.getEstimatedPosition().getY(), "RobotPoseY2");
    DataLogHelpers.logDouble(m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), "RobotPoseDeg2");
    DataLogHelpers.logDouble(m_field.getRobotPose().getX(), "FieldX");
    DataLogHelpers.logDouble(m_field.getRobotPose().getY(), "FieldY");
    DataLogHelpers.logDouble(m_field.getRobotPose().getRotation().getDegrees(), "FieldRot");
    SmartDashboard.putData("Field", m_field);
  }

  public SwerveModule getModule(int offset) {
    return m_modules[offset];
  }

/**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param m_fieldRelative Whether the provided x and y speeds are relative to the m_field.
   */
  public void controllerDrive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_pigeon2.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);

    for (int i = 0; i < 4; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
      DataLogHelpers.logDouble(swerveModuleStates[i].speedMetersPerSecond, "Swerve_" + i + "_drive");
      DataLogHelpers.logDouble(swerveModuleStates[i].angle.getDegrees(), "Swerve_" + i + "_angle");
    }
  }

  public void driveFieldSpeeds(ChassisSpeeds fieldSpeeds) {
    var relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      fieldSpeeds, getPose().getRotation());
    
    driveRobotSpeeds(relativeSpeeds);
  }

  public void driveRobotSpeeds(ChassisSpeeds robotSpeeds) {
    robotSpeeds.omegaRadiansPerSecond = -robotSpeeds.omegaRadiansPerSecond;
    var targetSpeeds = ChassisSpeeds.discretize(robotSpeeds, 0.02);
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(targetSpeeds);

    DataLogHelpers.logDouble(robotSpeeds.vxMetersPerSecond, "Vx");
    DataLogHelpers.logDouble(robotSpeeds.vyMetersPerSecond, "Vy");
    DataLogHelpers.logDouble(robotSpeeds.omegaRadiansPerSecond, "Omega");

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);

    for (int i = 0; i < 4; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update(m_pigeon2.getRotation2d(), getModulePositions());
  }

  public void addVisionMeasurements(Pose2d visionMeasurements, double timestamp) {
    m_poseEstimator.addVisionMeasurement(visionMeasurements, timestamp);
  }

  public Pose2d getPose() {
    // return m_odometry.getPoseMeters();
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d resetPose) {
    System.out.println("Reset pose to " + resetPose);
    m_poseEstimator.resetPosition(m_pigeon2.getRotation2d(), getModulePositions(), resetPose);
  }

  public void setGyro(double robotHeading) {
    m_pigeon2.setYaw(robotHeading);
  }

  public double getContinuousHeading() {
    return m_pigeon2.getYaw().getValue().in(Units.Degrees);
  }

  public Command resetGyro() {
    return new InstantCommand(() -> {m_pigeon2.reset();});
  }

  private void setBrakeMode(boolean enabled) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].setBrake(enabled);
    }
  }

  public Command setBrakeModeCmd() {
    return new InstantCommand(() -> setBrakeMode(true));
  }

  public Command setCoastModeCmd() {
    return new InstantCommand(() -> setBrakeMode(false));
  }

  public Command enableTurbo() {
      return new InstantCommand(() -> m_turbo = true);
  }

  public Command disableTurbo() {
      return new InstantCommand(() -> m_turbo = false);
  }

  public void setGains(double kp, double ki, double kd, double ks, double kv) {
    m_frontLeft.setGains(kp, ki, kd, ks, kv);
    m_frontRight.setGains(kp, ki, kd, ks, kv);
    m_backLeft.setGains(kp, ki, kd, ks, kv);
    m_backRight.setGains(kp, ki, kd, ks, kv);
  }

  public void homeSwerve() {
    System.out.println("Setting the zero position for the turning motors");
    for (int i = 0; i < 4; i++) {
      m_modules[i].syncSwerveEncoder(Constants.kZeroPosition[i]);
    }    
  }

  public void printHomePos() {
    double abspos[] = new double[4];
    for (int i = 0; i < 4; i++) { 
      abspos[i] = m_modules[i].getAbsPos();
      //System.out.println("abspos_" + i + ": " + abspos[i]);
    }
    SmartDashboard.putNumberArray("abspos", abspos);
  }

  public void rotateChassis() {
    driveRobotSpeeds(new ChassisSpeeds(0, 0, Constants.kAutoRotationSpeed));
  }

  public Command rotateChassisCmd(double rotationAngle) {
    return new FunctionalCommand(
      () -> {m_desiredAngle = rotationAngle + getPose().getRotation().getDegrees();}, 
      () -> {rotateChassis();},
      interrupted -> {}, // This should stop the arm at the current position
      () -> Math.abs(getPose().getRotation().getDegrees() - m_desiredAngle) < Constants.kTargetingError
    );
  }
}
