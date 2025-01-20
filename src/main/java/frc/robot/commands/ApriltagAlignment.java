package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** An example command that uses an example subsystem. */
public class ApriltagAlignment extends Command {
    private final int m_apriltag;
    private final double m_horizontalOffset;
    private final double m_distanceOffset;
    private final VisionSubsystem m_photonVision;
    private final DrivetrainSubsystem m_drivetrain;

    private final PIDController m_xPID = new PIDController(1.5, 0.2, 0.0);
    private final PIDController m_yPID = new PIDController(1.5, 0.2, 0.0);
    private final PIDController m_rotPID = new PIDController(1.5, 0.2, 0.0);

    private int m_counter = 0;

    public ApriltagAlignment(
        int apriltag,
        double horizontalOffset,
        double distanceOffset,
        VisionSubsystem photonVision,
        DrivetrainSubsystem drivetrain
    ) {
        m_apriltag = apriltag;
        m_horizontalOffset = horizontalOffset;
        m_distanceOffset = distanceOffset;
        m_photonVision = photonVision;

        m_drivetrain = drivetrain;
        addRequirements(drivetrain);

        m_xPID.setSetpoint(0.25);
        m_yPID.setSetpoint(0.15);
        m_rotPID.setSetpoint(Math.PI);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var target = m_photonVision.getTarget(m_apriltag);
        if (target.fiducialId == m_apriltag) {
            var x_offset = target.bestCameraToTarget.getMeasureX().baseUnitMagnitude();
            var y_offset = target.bestCameraToTarget.getMeasureY().baseUnitMagnitude();
            var rot = target.bestCameraToTarget.getRotation().getAngle();
            m_counter++;
            var x_drive = -m_xPID.calculate(x_offset);
            var y_drive = -m_yPID.calculate(y_offset);
            var rot_drive = m_rotPID.calculate(rot);
            System.out.println(m_counter + "=> " + x_offset + " " + y_offset + " " + rot);
            m_drivetrain.driveRobotSpeeds(new ChassisSpeeds(x_drive, y_drive, rot_drive));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
