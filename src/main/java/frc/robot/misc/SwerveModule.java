// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.utils.DataLogHelpers;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

public class SwerveModule {
  private static final double kModuleMaxAngularVelocity = Constants.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = Constants.kMaxAngularAcceleration;

  private int m_swerveIndex = -1;

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;
  private RelativeEncoder m_driveEncoder;
  private RelativeEncoder m_turningEncoder;

  //private final AnalogInput m_absolutePos;
  private final AnalogEncoder m_absolutePos;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(
    Constants.kDriveKp, Constants.kDriveKi, Constants.kDriveKd);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          Constants.kSwerveKp,
          Constants.kSwerveKi,
          Constants.kSwerveKd,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(
    Constants.kDriveKs, Constants.kDriveKv);
  
  private SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(
    Constants.kSwerveKs, Constants.kSwerveKv);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param driveEncoderChannelA DIO input for the drive encoder channel A
   * @param driveEncoderChannelB DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModule(
      int swerveIndex,
      int driveMotorChannel,
      int turningMotorChannel,
      int analogInputChannel) {
    m_swerveIndex = swerveIndex;
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    SparkMaxConfig driveConfig = new SparkMaxConfig();

    driveConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(Constants.NeoLimit);
    driveConfig.encoder
      .positionConversionFactor(2 * Math.PI * Constants.kWheelRadius / Constants.kDriveGearReduction)
      .velocityConversionFactor(2 * Math.PI * Constants.kWheelRadius / Constants.kDriveGearReduction / 60);
        
    m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig turnConfig = new SparkMaxConfig();

    turnConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Constants.NeoLimit);
      turnConfig.encoder
      .positionConversionFactor(2 * Math.PI / Constants.kTurningGearReduction)
      .velocityConversionFactor(2 * Math.PI / Constants.kTurningGearReduction / 60);

    m_turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_absolutePos = new AnalogEncoder(analogInputChannel);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();

    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // TODO: I think this needs to move into the PID controller of the spark max itself.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        -m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        -m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d encoderRotation = new Rotation2d(m_turningEncoder.getPosition());

    // Optimizing the state prevents rotations more than 90 degrees.
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.

    // This is only used in teleop (disabled in autonomous)
    // since it gives Pathplanner more precise translation control.
    if (DriverStation.isTeleop()) {
      desiredState.speedMetersPerSecond *= desiredState.angle.minus(encoderRotation).getCos();
    }

    // Calculate the drive output from the drive PID controller.
    final double drivePID =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), desiredState.speedMetersPerSecond);

    final double driveFF = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnPID =
      m_turningPIDController.calculate(m_turningEncoder.getPosition(), desiredState.angle.getRadians());

    final double turnFF = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.set(drivePID + driveFF);
    m_turningMotor.set(turnPID + turnFF);

    DataLogHelpers.logDouble(m_turningEncoder.getPosition(), "Swerve_rot_enc_" + m_swerveIndex);
    DataLogHelpers.logDouble(m_driveEncoder.getPosition(), "Swerve_drive_pos_" + m_swerveIndex);
    DataLogHelpers.logDouble(m_driveEncoder.getVelocity(), "Swerve_drive_vel_" + m_swerveIndex);
  }

  public void setGains(double kp, double ki, double kd, double ks, double kv) {
    m_turningPIDController.setP(kp);
    m_turningPIDController.setI(ki);
    m_turningPIDController.setD(kd);

    m_turnFeedforward = new SimpleMotorFeedforward(ks, kv);
  }

  private double getSwerveEncoderSyncedPos(double syncedAbsPos) {
    /* Calculate the error and wrap it around to the nearest half
    If the current rotation was 50, and the target was 4096, this
    would set the encoders rotation to ~+0.1 rotations instead of ~-12.5 */

    double diffPos = getAbsPos() - syncedAbsPos;
    if (diffPos < 0)
    {
      diffPos += 1;
    }
    // if (diffPos > Constants.kEncoderRes / 2)
    // {
    //   diffPos -= Constants.kEncoderRes;
    // }
    return (diffPos * 2 * Math.PI);// / Constants.kTurningGearReduction); /// Constants.kEncoderRes;
  }

  public void syncSwerveEncoder(double syncedAbsPos) {
    m_turningEncoder.setPosition(getSwerveEncoderSyncedPos(syncedAbsPos));
  }

  public double getAbsPos() {
    return m_absolutePos.get();
  }

  public void setBrake(boolean enabled) {
    if (enabled) {
      // TODO: Fixme
      //m_driveMotor.setIdleMode(IdleMode.kBrake);
    }
    else {
      // TODO: Fixme
      //m_driveMotor.setIdleMode(IdleMode.kCoast);
    }
  }
}
