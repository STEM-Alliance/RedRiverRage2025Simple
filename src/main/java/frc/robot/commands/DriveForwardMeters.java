package frc.robot.commands;

import static frc.robot.Constants.kMaxAlignmentAngularSpeed;
import static frc.robot.Constants.kMaxAlignmentSpeed;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** An example command that uses an example subsystem. */
public class DriveForwardMeters extends Command {
    private final DrivetrainSubsystem m_drivetrain;
    private final double m_initialDistance;
    private final double m_desiredDistance;

    private final PIDController m_distancePID = new PIDController(2.75, 0.0, 0.0);

    public DriveForwardMeters(double desiredDistance, DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        m_initialDistance = getSwerveFLCurrentDistance();
        m_desiredDistance = desiredDistance + m_initialDistance;

        addRequirements(m_drivetrain);
    }

    private final double getSwerveFLCurrentDistance() {
        return m_drivetrain.getModulePositions()[0].distanceMeters;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    public final ChassisSpeeds m_desiredChassisSpeeds = new ChassisSpeeds();

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_desiredChassisSpeeds.vxMetersPerSecond = 
            MathUtil.clamp(m_distancePID.calculate(getSwerveFLCurrentDistance(), m_desiredDistance), -0.2, 0.2);
        
        m_drivetrain.driveRobotSpeeds(m_desiredChassisSpeeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(getSwerveFLCurrentDistance() - m_desiredDistance) <= 0.05);
    }
}
