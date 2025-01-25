// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import frc.robot.commands.ApriltagAlignment;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.util.PathPlannerLogging;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> m_autoChooser;
  private final Field2d m_field = new Field2d();

  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem(m_field);
  private final VisionSubsystem m_photonVison = new VisionSubsystem(m_drivetrain.getPoseEstimator());
      
  private final CommandXboxController m_driverController = new CommandXboxController(kDriverControllerPort);

  public RobotContainer() {
    registerPathplannerCommands();
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous", m_autoChooser);
    configurePathplannerLogging();

    configureControllers();
  }

  private final void configureControllers() {
    //m_driverController.a();
    //m_driverController.b();
    m_driverController.x().onTrue(m_drivetrain.resetGyro());
    m_driverController.y().onTrue(new InstantCommand(() -> m_drivetrain.homeSwerve()));

    m_driverController.povLeft().whileTrue(new ApriltagAlignment(-1, 0.4, -0.15, m_photonVison, m_drivetrain));
    m_driverController.povRight().whileTrue(new ApriltagAlignment(-1, 0.4, 0.15, m_photonVison, m_drivetrain));

    // The drivetrain is responsible for the teleop drive command,
    // so this doesn't need to be changed between different drivetrains.
    m_drivetrain.setDefaultCommand(m_drivetrain.getTeleopDriveCommand(m_driverController));
    // new RunCommand(null, null).unless(
    //   new Trigger(
    //     () -> {
    //       double leftX = m_driverController.getLeftX();
    //       double leftY = m_driverController.getLeftY();
    //       double leftMagnitude = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));

    //       return (leftMagnitude > kControllerDeadband);
    //     }
    //   )
    // );
  }

  private final void configurePathplannerLogging() {
    // Logging callback for the active path.
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      m_field.getObject("path").setPoses(poses);
    });

    // Logging callback for the current pose.
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      m_field.setRobotPose(pose);
    });

    // Logging callback for the target pose.
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      m_field.getObject("target pose").setPose(pose);
    });

    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Registers the named commands for the Pathplanner autonomous modes. This needs to be called
   * after the {@link AutoBuilder} is created, but before the Pathplanner auto chooser is built.
  */
  private final void registerPathplannerCommands() {
    NamedCommands.registerCommand("AlignLeftOffset", new ApriltagAlignment(-1, 0.4, -0.15, m_photonVison, m_drivetrain));
    NamedCommands.registerCommand("AlignRightOffset", new ApriltagAlignment(-1, 0.4, 0.15, m_photonVison, m_drivetrain));
  }

  /**
   * Gets the autonomous command from Pathplanner for the {@link Robot#autonomousInit()}.
   * If no autonomous mode is chosen, this will return an empty {@link InstantCommand}.
   */
  public final Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
