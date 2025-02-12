// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import frc.robot.commands.ApriltagAlignment;
import frc.robot.commands.ApriltagOverride;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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

  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem(42, 43);
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem(m_field);
  private final VisionSubsystem m_photonVison = new VisionSubsystem(m_drivetrain.getPoseEstimator());

  private final ClimbSubsystem m_climb = new ClimbSubsystem();
      
  private final CommandXboxController m_driverController = new CommandXboxController(kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(kOperatorControllerPort);
  private final CommandJoystick m_operatorButtonPanel = new CommandJoystick(kOperatorButtonPanelPort);

  public RobotContainer() {
    registerPathplannerCommands();
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous", m_autoChooser);
    configurePathplannerLogging();

    configureControllers();
  }

  private final void configureControllers() {
    // m_driverController.a();
    // m_driverController.b();
    // m_driverController.x();
    m_driverController.y().onTrue(m_drivetrain.resetGyro());

    m_driverController.leftBumper().onTrue(m_climb.toggleClimber());
    m_driverController.rightBumper().onTrue(m_climb.toggleClaw());

    m_driverController.leftTrigger().whileTrue(new ApriltagAlignment(-1, 0.8, -0.15, m_photonVison, m_drivetrain, true));
    m_driverController.rightTrigger().whileTrue(new ApriltagAlignment(-1, 0.8, 0.15, m_photonVison, m_drivetrain, true));

    // The drivetrain is responsible for the teleop drive command,
    // so this doesn't need to be changed between different drivetrains.
    m_drivetrain.setDefaultCommand(m_drivetrain.getTeleopDriveCommand(m_driverController));

    // L4
    m_operatorButtonPanel.button(2).onTrue(m_elevator.setState(
      2,
      kElevatorSetpoints.L4,
      kShooterSetpoints.L4
    )).onFalse(m_elevator.setStateIdle(2));

    // L3
    m_operatorButtonPanel.button(4).onTrue(m_elevator.setState(
      4,
      kElevatorSetpoints.L3,
      kShooterSetpoints.L3
    )).onFalse(m_elevator.setStateIdle(4));

    // L2
    m_operatorButtonPanel.button(6).onTrue(m_elevator.setState(
      6,
      kElevatorSetpoints.L2,
      kShooterSetpoints.L2
    )).onFalse(m_elevator.setStateIdle(6));

    // L1
    m_operatorButtonPanel.button(8).onTrue(m_elevator.setState(
      8,
      kElevatorSetpoints.L1,
      kShooterSetpoints.L1
    )).onFalse(m_elevator.setStateIdle(8));

    // // Intake
    // m_operatorButtonPanel.button(9).onTrue();
    // m_operatorButtonPanel.button(9).onFalse();

    // // Shoot
    // m_operatorButtonPanel.button(10).onTrue();
    // m_operatorButtonPanel.button(10).onFalse();

    // m_driverController
    //     .a()
    //     .and(m_driverController.rightBumper())
    //     .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_driverController
    //     .b()
    //     .and(m_driverController.rightBumper())
    //     .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_driverController
    //     .x()
    //     .and(m_driverController.rightBumper())
    //     .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_driverController
    //     .y()
    //     .and(m_driverController.rightBumper())
    //     .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
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
    NamedCommands.registerCommand("AlignLeftInterrupt", new ApriltagOverride(-1, 0.8, -0.15, m_photonVison, m_drivetrain));
    NamedCommands.registerCommand("AlignLeftOffset", new ApriltagAlignment(-1, 0.8, -0.15, m_photonVison, m_drivetrain, true));

    NamedCommands.registerCommand("AlignRightInterrupt", new ApriltagOverride(-1, 0.8, 0.15, m_photonVison, m_drivetrain));
    NamedCommands.registerCommand("AlignRightOffset", new ApriltagAlignment(-1, 0.8, 0.15, m_photonVison, m_drivetrain, true));
  }

  /**
   * Gets the autonomous command from Pathplanner for the {@link Robot#autonomousInit()}.
   * If no autonomous mode is chosen, this will return an empty {@link InstantCommand}.
   */
  public final Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
