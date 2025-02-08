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
        ChassisSpeeds current = m_drivetrain.getChassisSpeeds();
        ChassisSpeeds alignment = m_alignmentCommand.m_desiredChassisSpeeds;

        // This counter will count up even if a new frame is not recieved. It will at least
        // ensure that there is a delay between the first detection and the interrupt.
        if ((m_counter >= 4)) {
            // The squared speed is used to avoid unnecessary square roots, the real speed doesn't matter.
            double currentSpeedSquared = Math.pow(current.vxMetersPerSecond, 2) + Math.pow(current.vyMetersPerSecond, 2);
            double alignmentSpeedSquared = Math.pow(alignment.vxMetersPerSecond, 2) + Math.pow(alignment.vyMetersPerSecond, 2);
            
            return ((currentSpeedSquared <= alignmentSpeedSquared)) && (alignmentSpeedSquared <= Math.pow(kMaxAutonomousSpeed, 2));
        }

        return false;
    }
}
