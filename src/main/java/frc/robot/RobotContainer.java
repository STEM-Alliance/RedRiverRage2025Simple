// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import org.photonvision.PhotonCamera;

import frc.robot.Constants.kElevatorSetpoints;
import frc.robot.Constants.kShooterSetpoints;
import frc.robot.commands.ApriltagAlignment;
import frc.robot.commands.ApriltagOverride;
import frc.robot.commands.DriveForwardMeters;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Elastic;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> m_autoChooser;
  private final Field2d m_field = new Field2d();

  private final CommandXboxController m_driverController = new CommandXboxController(kDriverControllerPort);
  private final CommandJoystick m_operatorButtonPanel = new CommandJoystick(kOperatorButtonPanelPort);

  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem(12, 10);
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem(m_field);

  private final ClimbSubsystem m_climb = new ClimbSubsystem(15, 14, 13, 12);
  private final IntakeSubsystem m_intake = new IntakeSubsystem(16, 0, 1, m_driverController);

  private final VisionSubsystem[] m_cameras = new VisionSubsystem[]{
    new VisionSubsystem(
      "FrontCAM",
      new Transform3d(
        new Translation3d(0.27305, -0.076, 0.222),
        new Rotation3d(0.0, -16.0 * (Math.PI / 180.0), 0.0) // 16 degrees vertical, negative
      ),

      m_drivetrain.getPoseEstimator()
    )
  };

  // elevator is 12, rotation is 10, shooter is 11
  // shooter and elevator brushless, rotation brushed

  public RobotContainer() {
    registerPathplannerCommands();
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous", m_autoChooser);
    configurePathplannerLogging();

    configureControllers();
    SmartDashboard.putBoolean("FinishedAligning", true);
  }

  public final DrivetrainSubsystem getDrivetrain() {
    return m_drivetrain;
  }

  private final void configureControllers() {
    m_driverController.a().whileTrue(m_drivetrain.driveStraight(1, false));
    m_driverController.b().whileTrue(m_drivetrain.driveStraight(1.0, true));
    m_driverController.x().onTrue(m_drivetrain.resetGyro());
    m_driverController.y().onTrue(m_drivetrain.calculateWheelDiameters(m_driverController, m_driverController.povUp()));

    m_driverController.leftBumper().onTrue(m_climb.toggleClimber().alongWith(m_elevator.setState(kElevatorSetpoints.IDLE, kShooterSetpoints.CLIMB)).alongWith(m_drivetrain.toggleClimbSpeed()).alongWith(new InstantCommand(() -> {
      Elastic.selectTab("Climb");
    })));

    m_driverController.rightBumper().onTrue(m_climb.toggleClaw());

    m_driverController.leftTrigger()
      .whileTrue(new ApriltagAlignment(-1, -1, Constants.kAlignYDistanceLeft, m_cameras, m_drivetrain, true)
      .andThen(new ApriltagAlignment(-1, Constants.kAlignXDistance, -1, m_cameras, m_drivetrain, true)
      .andThen(new DriveForwardMeters(1.45, m_drivetrain))));
    m_driverController.rightTrigger()
      .whileTrue(new ApriltagAlignment(-1, -1, Constants.kAlignYDistanceRight, m_cameras, m_drivetrain, true)
      .andThen(new ApriltagAlignment(-1, Constants.kAlignXDistance, -1, m_cameras, m_drivetrain, true)
      .andThen(new DriveForwardMeters(1.45, m_drivetrain))));
    // m_driverController.leftTrigger().whileTrue(new ApriltagAlignment(-1, Constants.kAlignXDistance, Constants.kAlignYDistanceLeft, m_cameras, m_drivetrain, true).andThen(new DriveForwardMeters(1.45, m_drivetrain)));
    // m_driverController.rightTrigger().whileTrue(new ApriltagAlignment(-1, Constants.kAlignXDistance, Constants.kAlignYDistanceRight, m_cameras, m_drivetrain, true).andThen(new DriveForwardMeters(1.45, m_drivetrain)));

    //m_driverController.leftBumper().whileTrue(m_elevator.cw());
    //m_driverController.rightBumper().whileTrue(m_elevator.ccw());
    //m_driverController.b().whileTrue(m_elevator.up());
    //m_driverController.a().whileTrue(m_elevator.down());
    // m_driverController.povLeft().whileTrue(m_intake.startShooting());
    // m_driverController.povRight().whileTrue(m_elevator.ccw());
    // m_driverController.povUp().onTrue(m_intake.startIntaking());
    // m_driverController.povDown().onTrue(m_intake.stopIntaking());
    // m_driverController.a().onTrue(m_elevator.setState(kElevatorSetpoints.L1, kShooterSetpoints.L1));
    // m_driverController.b().onTrue(m_elevator.setState(kElevatorSetpoints.L2, kShooterSetpoints.L2));
    // m_driverController.x().onTrue(m_elevator.setState(kElevatorSetpoints.L3, kShooterSetpoints.L3));
    // m_driverController.start().onTrue(m_elevator.setState(kElevatorSetpoints.L4, kShooterSetpoints.L4));

    // The drivetrain is responsible for the teleop drive command,
    // so this doesn't need to be changed between different drivetrains.
    m_drivetrain.setDefaultCommand(m_drivetrain.getTeleopDriveCommand(m_driverController));

    m_operatorButtonPanel.button(1).onTrue(m_elevator.setState(
      kElevatorSetpoints.L1,
      kShooterSetpoints.L1
    )).onFalse(m_elevator.setStateIdle());

    m_operatorButtonPanel.button(2).onTrue(m_elevator.setState(
      kElevatorSetpoints.L2,
      kShooterSetpoints.L2
    )).onFalse(m_elevator.setStateIdle());

    m_operatorButtonPanel.button(3).onTrue(m_elevator.setState(
      kElevatorSetpoints.L3,
      kShooterSetpoints.L3
    )).onFalse(m_elevator.setStateIdle());

    m_operatorButtonPanel.button(4).onTrue(m_elevator.setState(
      kElevatorSetpoints.L4,
      kShooterSetpoints.L4
    )).onFalse(m_elevator.setStateIdle());

    m_operatorButtonPanel.button(6).onTrue(m_intake.startIntaking().alongWith(m_elevator.setState(kElevatorSetpoints.INTAKE, kShooterSetpoints.INTAKE)))
      .onFalse(m_intake.stopIntaking().alongWith(m_elevator.setState(kElevatorSetpoints.IDLE, kShooterSetpoints.IDLE)));
    
    m_operatorButtonPanel.button(5).onTrue(m_intake.startShooting())
      .onFalse(m_intake.stopShooting());
    
    m_operatorButtonPanel.button(8).whileTrue(m_elevator.zeroHeight())
      .onFalse(m_elevator.emergencyStop());
    
      m_operatorButtonPanel.button(7).onTrue(m_intake.startIntaking().alongWith(m_elevator.setState(kElevatorSetpoints.INTAKE, kShooterSetpoints.INTAKE2)))
      .onFalse(m_intake.stopIntaking().alongWith(m_elevator.setState(kElevatorSetpoints.IDLE, kShooterSetpoints.IDLE)));

    // m_operatorButtonPanel.button(4).onTrue(m_elevator.setState(
    //   kElevatorSetpoints.L3,
    //   kShooterSetpoints.L3
    // )).onFalse(m_elevator.setStateIdle());

    // m_operatorButtonPanel.button(6).onTrue(m_elevator.setState(`
    //   kElevatorSetpoints.L2,
    //   kShooterSetpoints.L2
    // )).onFalse(m_elevator.setStateIdle());

    // m_operatorButtonPanel.button(8).onTrue(m_elevator.setState(
    //   kElevatorSetpoints.L1,
    //   kShooterSetpoints.L1
    // )).onFalse(m_elevator.setStateIdle());

    // m_operatorButtonPanel.button(9).onTrue(
    //   m_elevator.startIntaking()
    // ).onFalse(m_elevator.stopIntaking());

    // m_operatorButtonPanel.button(10).onTrue(
    //   m_elevator.startShooting()
    // ).onFalse(m_elevator.stopShooting());

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
    //     .and(m_driverController.rightBumper()).


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
    NamedCommands.registerCommand("AlignLeftInterrupt", new ApriltagOverride(-1, Constants.kAlignXDistance, Constants.kAlignYDistanceLeft, m_cameras, null));
    NamedCommands.registerCommand("AlignLeftOffset", new ApriltagAlignment(-1, Constants.kAlignXDistance, Constants.kAlignYDistanceLeft, m_cameras, m_drivetrain, true));

    NamedCommands.registerCommand("AlignRightInterrupt", new ApriltagOverride(-1, Constants.kAlignXDistance, Constants.kAlignYDistanceRight, m_cameras, null));
    NamedCommands.registerCommand("AlignRightOffset", new ApriltagAlignment(-1, Constants.kAlignXDistance, Constants.kAlignYDistanceRight, m_cameras, m_drivetrain, true));
    NamedCommands.registerCommand("DriveForwardMeters", new DriveForwardMeters(1.55, m_drivetrain));
    NamedCommands.registerCommand("Stop", stopAuto());

    NamedCommands.registerCommand("SetState L4", m_elevator.setState(kElevatorSetpoints.L4, kShooterSetpoints.L4)
      .alongWith(m_elevator.atSetpoints()));
    NamedCommands.registerCommand("Shoot 3 Sec", m_intake.startShooting().andThen(new WaitCommand(1.0))
      .andThen(m_intake.stopShooting()));
    NamedCommands.registerCommand("SetState IDLE", m_elevator.setState(kElevatorSetpoints.IDLE, kShooterSetpoints.IDLE));
    NamedCommands.registerCommand("Just SetState INTAKE", m_elevator.setState(kElevatorSetpoints.INTAKE, kShooterSetpoints.INTAKE));
    NamedCommands.registerCommand("SetState INTAKE", m_elevator.setState(kElevatorSetpoints.INTAKE, kShooterSetpoints.INTAKE).andThen(m_intake.startIntaking())
      .andThen(m_elevator.setState(kElevatorSetpoints.IDLE, kShooterSetpoints.IDLE)));

    NamedCommands.registerCommand("Shoot", m_intake.startShooting().until(new Trigger(m_intake::isIntakeNotLoaded)));
    NamedCommands.registerCommand("Intake", m_intake.runIntake());
  }

  /**
   * Gets the autonomous command from Pathplanner for the {@link Robot#autonomousInit()}.
   * If no autonomous mode is chosen, this will return an empty {@link InstantCommand}.
   */
  public final Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  private final Command stopAuto() {
    return new FunctionalCommand(
      () -> {},
      () -> {m_drivetrain.driveRobotSpeeds(new ChassisSpeeds());},
      interrupted -> {},
      () -> false,
      m_drivetrain
    );
  }
}
