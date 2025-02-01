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

    private final PIDController m_xPID = new PIDController(1.75, 0.0, 0.0);
    private final PIDController m_yPID = new PIDController(1.75, 0.0, 0.0);
    private final PIDController m_rotPID = new PIDController(2.0, 0.2, 0.0);

    private int m_counter = 0;
    private int m_apriltag;
    private boolean m_output;

    public ApriltagAlignment(
        int apriltag,
        double xOffset,
        double yOffset,
        VisionSubsystem photonVision,
        DrivetrainSubsystem drivetrain,
        boolean output
    ) {
        m_output = output;
        m_apriltag = apriltag;
        m_photonVision = photonVision;

        m_drivetrain = drivetrain;

        m_xPID.setSetpoint(xOffset);
        m_yPID.setSetpoint(yOffset);
        m_rotPID.setSetpoint(Math.PI);
        m_xPID.setTolerance(0.05);
        m_yPID.setTolerance(0.05);
        m_rotPID.setTolerance(0.05);

        // Integral is only used within +- 12.5 degrees of the target, with -0.1 to 0.1 max influence.
        m_rotPID.setIZone(12.5);
        m_rotPID.setIntegratorRange(-0.075, 0.075);

        if (output) addRequirements(drivetrain);

        System.out.println("+ApriltagAlignment command");
    }

    public final void disableOutput() {
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_apriltag = -1;
    }

    public final ChassisSpeeds m_desiredChassisSpeeds = new ChassisSpeeds();

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("+ApriltagAlignment.execute");
        var target = m_photonVision.getTargetClosestToCenter();

        // TODO: If the target is lost, the robot will keep rotating.
        // Also, if moving too fast then it can reach the setpoint and then overshoot.
        if (target != null) {
            m_apriltag = target.fiducialId;
            var x_offset = target.bestCameraToTarget.getMeasureX().baseUnitMagnitude();
            var y_offset = target.bestCameraToTarget.getMeasureY().baseUnitMagnitude();
            var rot = target.bestCameraToTarget.getRotation().getAngle();
            m_desiredChassisSpeeds.vxMetersPerSecond = -m_xPID.calculate(x_offset);
            m_desiredChassisSpeeds.vyMetersPerSecond = -m_yPID.calculate(y_offset);
            m_desiredChassisSpeeds.omegaRadiansPerSecond = m_rotPID.calculate(rot);

            if (m_output) m_drivetrain.driveRobotSpeeds(m_desiredChassisSpeeds);
            
            m_counter++;
            System.out.println(m_counter + "=> " + x_offset + " " + y_offset + " " + rot);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {m_apriltag = -1;}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_rotPID.atSetpoint() && m_apriltag > 0)
        {
            System.out.println("ApriltagAlignment isFinished");
            return true;
        }
        return false;
    }
}
