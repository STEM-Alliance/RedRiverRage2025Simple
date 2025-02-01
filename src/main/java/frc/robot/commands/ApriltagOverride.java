package frc.robot.commands;

import static frc.robot.Constants.kMaxAutonomousSpeed;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ApriltagOverride extends Command {
    private final VisionSubsystem m_photonVision;
    private final DrivetrainSubsystem m_drivetrain;

    private int m_counter = 0;
    private ApriltagAlignment m_alignmentCommand;

    public ApriltagOverride(
        int apriltag,
        double xOffset,
        double yOffset,
        VisionSubsystem photonVision,
        DrivetrainSubsystem drivetrain
    ) {
        m_photonVision = photonVision;
        m_drivetrain = drivetrain;

        m_alignmentCommand = new ApriltagAlignment(apriltag, xOffset, yOffset, photonVision, drivetrain, false);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_alignmentCommand.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var target = m_photonVision.getTargetClosestToCenter();

        if (target != null) {
            m_counter++;
        }

        else {
            m_counter = 0;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_alignmentCommand.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        ChassisSpeeds a = m_drivetrain.getChassisSpeeds();
        ChassisSpeeds b = m_alignmentCommand.m_desiredChassisSpeeds;

        if ((m_counter >= 10) && Math.abs(a.omegaRadiansPerSecond) <= Math.abs(b.omegaRadiansPerSecond)) {
            // While the direction being travelled is ignored, it can be assumed that it will be in a similar direction.
            var aMagnitude = Math.sqrt(Math.pow(a.vxMetersPerSecond, 2) + Math.pow(a.vyMetersPerSecond, 2));
            var bMagnitude = Math.sqrt(Math.pow(b.vxMetersPerSecond, 2) + Math.pow(b.vyMetersPerSecond, 2));
            
            return (aMagnitude <= bMagnitude) && !(bMagnitude > Math.sqrt(2 * Math.pow(kMaxAutonomousSpeed, 2)));
        }

        return false;
    }
}
