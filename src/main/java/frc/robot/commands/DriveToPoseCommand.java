package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToPoseCommand extends Command {
    private final Pose2d m_goal;
    private final DrivetrainSubsystem m_drivetrain;

    private final PIDController m_xPID = new PIDController(1.5, 0.0, 0.0);
    private final PIDController m_yPID = new PIDController(1.5, 0.0, 0.0);
    private final PIDController m_omegaPID = new PIDController(2.5, 0.0, 0.0);

    private final ChassisSpeeds m_desiredSpeeds = new ChassisSpeeds();

    public DriveToPoseCommand(Pose2d goal, DrivetrainSubsystem drivetrain) {
        m_goal = goal;
        m_drivetrain = drivetrain;

        m_xPID.setSetpoint(m_goal.getX());
        m_yPID.setSetpoint(m_goal.getY());
        m_omegaPID.setSetpoint(m_goal.getRotation().getRadians());

        m_xPID.setTolerance(0.025);
        m_yPID.setTolerance(0.025);
        m_omegaPID.setTolerance(Units.degreesToRadians(2.5));

        m_omegaPID.enableContinuousInput(0, Units.degreesToRadians(180));

        addRequirements(m_drivetrain);
    }

    @Override
    public final void initialize() {}

    @Override
    public final void execute() {
        Pose2d currentPose = m_drivetrain.getPose();

        m_desiredSpeeds.vxMetersPerSecond = MathUtil.clamp(m_xPID.calculate(currentPose.getX()), -1.5, 1.5);
        m_desiredSpeeds.vyMetersPerSecond = MathUtil.clamp(m_yPID.calculate(currentPose.getY()), -1.5, 1.5);
        m_desiredSpeeds.omegaRadiansPerSecond = MathUtil.clamp(m_omegaPID.calculate(currentPose.getRotation().getRadians()), -Math.PI, Math.PI);

        m_drivetrain.driveFieldSpeeds(m_desiredSpeeds);
    }

    @Override
    public final void end(boolean interrupted) {}

    @Override
    public final boolean isFinished() {
        return (m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_omegaPID.atSetpoint());
    }
}
