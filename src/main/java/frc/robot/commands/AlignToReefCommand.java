package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AprilTagFieldHelpers;

public class AlignToReefCommand extends Command {
    private final Transform2d m_alignmentTransform;
    private final DrivetrainSubsystem m_drivetrain;

    private DriveToPoseCommand m_driveToPoseCommand;

    public AlignToReefCommand(double xOffset, double yOffset, DrivetrainSubsystem drivetrain) {
        m_alignmentTransform = new Transform2d(xOffset, yOffset, Rotation2d.kZero);
        m_drivetrain = drivetrain;
    }

    @Override
    public final void initialize() {
        m_driveToPoseCommand = new DriveToPoseCommand(
            getGoalPose(), m_drivetrain
        );

        m_driveToPoseCommand.schedule();
    }

    @Override
    public final void execute() {}

    @Override
    public final void end(boolean interrupted) {}

    @Override
    public final boolean isFinished() {
        return !m_driveToPoseCommand.isScheduled();
    }

    private final Pose2d getGoalPose() {
        // When the robot is 180 from the tag, we need to offset the y distance since
        // the shooter is not centered on the robot.
        return AprilTagFieldHelpers.getNearestReefPose(m_drivetrain.getPose())
            .transformBy(m_alignmentTransform);
    }
}
