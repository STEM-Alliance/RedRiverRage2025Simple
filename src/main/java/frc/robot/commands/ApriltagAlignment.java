package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** An example command that uses an example subsystem. */
public class ApriltagAlignment extends Command {
    private final VisionSubsystem m_photonVision;
    private final DrivetrainSubsystem m_drivetrain;

    private final PIDController m_xPID = new PIDController(1.5, 0.2, 0.0);
    private final PIDController m_yPID = new PIDController(1.5, 0.2, 0.0);
    private final PIDController m_rotPID = new PIDController(1.5, 0.2, 0.0);

    private int m_counter = 0;
    private int m_apriltag;

    public ApriltagAlignment(
        int apriltag,
        double horizontalOffset,
        double distanceOffset,
        VisionSubsystem photonVision,
        DrivetrainSubsystem drivetrain
    ) {
        m_apriltag = apriltag;
        m_photonVision = photonVision;

        m_drivetrain = drivetrain;
        addRequirements(drivetrain);

        m_xPID.setSetpoint(horizontalOffset);
        m_yPID.setSetpoint(distanceOffset);
        m_rotPID.setSetpoint(Math.PI);
        m_xPID.setTolerance(0.05);
        m_yPID.setTolerance(0.05);
        m_rotPID.setTolerance(0.05);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var target = m_photonVision.getCentralTarget();
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

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {m_apriltag = -1;}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_rotPID.atSetpoint())
            return true;
        return false;
    }
}
