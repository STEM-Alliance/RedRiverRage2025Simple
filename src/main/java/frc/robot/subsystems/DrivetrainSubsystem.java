// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.ControllerProcessing;
import frc.robot.utils.DataLogHelpers;
import frc.robot.Constants;
import frc.robot.misc.SwerveModule;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
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

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return m_poseEstimator;
  }

  double m_desiredAngle = 0;

  private RobotConfig m_robotConfig;
  private final Alert m_pathplannerRobotConfigAlert = new Alert(
    "DrivetrainSubsystem: Pathplanner failed to load the robot configuration!",
    AlertType.kError
  );

  private final Field2d m_field;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              voltage -> {
                driveLeftVoltage(voltage);
                driveRightVoltage(voltage);
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            leftMotorGet() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(leftMotorEncoderDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(leftMotorEncoderRate(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            rightMotorGet() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(rightMotorEncoderDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(rightMotorEncoderRate(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  
    /** Creates a new DriveSubSystem. */
    public DrivetrainSubsystem(Field2d field) {
      m_pigeon2.reset();
      m_field = field;
  
      // Set deviations for the pose estimator vision measurements, rotation is positive infinity since
      // the gyro will give us more accurate results than the vision system. We should also scale this by the
      // distance of the tag detected, longer distances will be less accurate so will have less of an effect
  
      // Actual deviations are 0.5, over time odometry becomes less accurate?
      // Deviaton for rotation should be Double.POSITIVE_INFINITY
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(1.0, 1.0, 10.0));
  
      try {
        m_robotConfig = RobotConfig.fromGUISettings();
        m_pathplannerRobotConfigAlert.close();
      }
      
      catch (Exception e) {
        m_pathplannerRobotConfigAlert.set(true);
        // TODO: I am unsure what effects this will have, should this just disable autonomous?
        m_robotConfig = new RobotConfig(null, null, null, null, null);
    }

    AutoBuilder.configure(
      this::getPose, // Pose2d supplier
      this::resetPose, // Method to reset odometry
      this::getChassisSpeeds, // Robot-relative ChassisSpeeds supplier
      (speeds, feedforwards) -> driveRobotSpeeds(speeds), // Robot-relative driving with optional feedfowards.

      new PPHolonomicDriveController(
        new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(2.25, 0.0, 0.0) // Rotation PID constants
      ),

      m_robotConfig, // Robot hardware configuration

      () -> {
        // Boolean supplier that controls when the path is mirrored for the red alliance.
        // This will flip the path being followed to the red side of the field, but the
        // origin will remain on the blue side of the field.
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) {
          // By default, the path will not be mirrored.
          return false;
        }

        return alliance.get() == DriverStation.Alliance.Red;
      },

      this
    );
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
    SmartDashboard.putNumber("RobotX", getPose().getX());
    SmartDashboard.putNumber("RobotY", getPose().getY());
  }

  public final Command getTeleopDriveCommand(CommandXboxController driverController) {
    return new RunCommand(
      () -> {
        // Since the field is sideways (in the coordinate system) relative to the driver station,
        // the x and y coordinates are flipped (x is sideways on the joystick, vertical on the field).
        double[] processedXY = ControllerProcessing.getProcessedTranslation(
          driverController.getLeftY(), driverController.getLeftX()
        );

        double processedOmega = ControllerProcessing.getProcessedOmega(driverController.getRightX());

        controllerDrive(
          processedXY[0] * kMaxSpeed,
          processedXY[1] * kMaxSpeed,
          processedOmega * kMaxAngularSpeed,
          true,
          0.02
        );
      },
      this
    );
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
    // TODO: Make the pathplanner GUI speed limit actually limit speed
    // Use the robot config module config max speeds
    // Should there be a seperate function for pathplanner drive?
    robotSpeeds.vxMetersPerSecond = -robotSpeeds.vxMetersPerSecond;
    robotSpeeds.vyMetersPerSecond = -robotSpeeds.vyMetersPerSecond;
    robotSpeeds.omegaRadiansPerSecond = -robotSpeeds.omegaRadiansPerSecond;

    if (DriverStation.isAutonomous()) {
      double maxDriveVelocity = m_robotConfig.moduleConfig.maxDriveVelocityMPS;
      double maxRotationalVelocity = m_robotConfig.moduleConfig.maxDriveVelocityRadPerSec;

      robotSpeeds.vxMetersPerSecond = MathUtil.clamp(
        robotSpeeds.vxMetersPerSecond, -maxDriveVelocity, maxDriveVelocity
      );

      robotSpeeds.vyMetersPerSecond = MathUtil.clamp(
        robotSpeeds.vyMetersPerSecond, -maxDriveVelocity, maxDriveVelocity
      );

      robotSpeeds.omegaRadiansPerSecond = MathUtil.clamp(
        robotSpeeds.omegaRadiansPerSecond, -maxRotationalVelocity, maxRotationalVelocity
      );
    }

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

  public void driveLeftVoltage(Voltage volts)
  {
    m_frontLeft.getDriveMotor().setVoltage(volts);
    m_backLeft.getDriveMotor().setVoltage(volts);
  }

  public void driveRightVoltage(Voltage volts)
  {
    m_frontRight.getDriveMotor().setVoltage(volts);
    m_backRight.getDriveMotor().setVoltage(volts);
  }

  public double leftMotorGet()
  {
    double frontLeftVelocity = m_frontLeft.getDriveMotor().getAppliedOutput();
    double backLeftVelocity = m_backLeft.getDriveMotor().getAppliedOutput();

    return (frontLeftVelocity + backLeftVelocity) / 2.0;
  }

  public double rightMotorGet()
  {
    double frontRightVelocity = m_frontRight.getDriveMotor().getAppliedOutput();
    double backRightVelocity = m_backRight.getDriveMotor().getAppliedOutput();

    return (frontRightVelocity + backRightVelocity) / 2.0;
  }

  public double leftMotorEncoderDistance()
  {
    double frontLeftDistance = m_frontLeft.getDriveMotor().getEncoder().getPosition();
    double backLeftDistance = m_backLeft.getDriveMotor().getEncoder().getPosition();

    return (frontLeftDistance + backLeftDistance) / 2.0;
  }

  public double rightMotorEncoderDistance()
  {
    double frontRightDistance = m_frontRight.getDriveMotor().getEncoder().getPosition();
    double backRightDistance = m_backRight.getDriveMotor().getEncoder().getPosition();

    return (frontRightDistance + backRightDistance) / 2.0;
  }

  public double leftMotorEncoderRate()
  {
    double frontLeftVelocity = m_frontLeft.getDriveMotor().getEncoder().getVelocity();
    double backLeftVelocity = m_backLeft.getDriveMotor().getEncoder().getVelocity();

    return (frontLeftVelocity + backLeftVelocity) / 2.0;
  }

  public double rightMotorEncoderRate()
  {
    double frontRightVelocity = m_frontRight.getDriveMotor().getEncoder().getVelocity();
    double backRightVelocity = m_backRight.getDriveMotor().getEncoder().getVelocity();

    return (frontRightVelocity + backRightVelocity) / 2.0;
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
