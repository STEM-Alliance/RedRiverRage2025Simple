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
import frc.robot.util.DataLogHelpers;

/** An example command that uses an example subsystem. */
public class ApriltagAlignment extends Command {
    // TODO: This doesnt account for the camera offset
    private final VisionSubsystem m_photonVision;
    private final DrivetrainSubsystem m_drivetrain;

    private final PIDController m_xPID = new PIDController(3.5, 0.0, 0.0);

    private final PIDController m_yPID = new PIDController(3.5, 0.0, 0.0);

    private final PIDController m_rotPID = new PIDController(2.25, 0.2, 0.0);

    private int m_counter = 0;
    private int m_apriltag;
    private boolean m_output;
    private boolean m_activeX;
    private boolean m_activeY;

    public ApriltagAlignment(
        int apriltag,
        double xOffset,
        double yOffset,
        VisionSubsystem[] cameras,
        DrivetrainSubsystem drivetrain,
        boolean output
    ) {
        m_output = output;
        m_apriltag = apriltag;
        m_photonVision = cameras[0];

        m_drivetrain = drivetrain;

        if (xOffset < 0)
            m_activeX = false;
        else
            m_activeX = true;

        if (yOffset < 0)
            m_activeY = false;
        else
            m_activeY = true;

        m_xPID.setSetpoint(xOffset);
        m_yPID.setSetpoint(yOffset);
        m_rotPID.setSetpoint(Math.PI);
        m_xPID.setTolerance(0.05);
        m_yPID.setTolerance(0.05);
        m_rotPID.setTolerance(0.025);

        // Integral is only used within +- 12.5 degrees of the target, with -0.1 to 0.1 max influence.
        m_rotPID.setIZone(12.5);
        m_rotPID.setIntegratorRange(-0.075, 0.075);

        if (output) addRequirements(drivetrain);

        System.out.println("+ApriltagAlignment command");

        SmartDashboard.putData("AT_x_pid", m_xPID);
        SmartDashboard.putData("AT_y_pid", m_yPID);
        SmartDashboard.putData("AT_rot_pid", m_rotPID);
    }

    public final void disableOutput() {
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_apriltag = -1;
        m_photonVision.getCamera().takeOutputSnapshot();
        SmartDashboard.putBoolean("FinishedAligning", false);
    }

    public final ChassisSpeeds m_desiredChassisSpeeds = new ChassisSpeeds();

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        DataLogHelpers.logDouble(1.0, "AlignmentStatus");
        System.out.println("+ApriltagAlignment.execute");
        var target = m_photonVision.getTargetClosestToCenter();

        // TODO: If the target is lost, the robot will keep rotating.
        // Also, if moving too fast then it can reach the setpoint and then overshoot.
        if (target != null) {
            SmartDashboard.putNumber("AT_alignment_tag_id", target.fiducialId);
            SmartDashboard.putNumber("AT_alignment_tag_x", target.bestCameraToTarget.getX());
            SmartDashboard.putNumber("AT_alignment_tag_y", target.bestCameraToTarget.getY());
            SmartDashboard.putNumber("AT_alignment_tag_rot", target.bestCameraToTarget.getRotation().getAngle());
            SmartDashboard.putBoolean("AT_alignment_tag_AtX", m_yPID.atSetpoint());
            SmartDashboard.putBoolean("AT_alignment_tag_AtY", m_yPID.atSetpoint());
            SmartDashboard.putBoolean("AT_alignment_tag_AtRot", m_rotPID.atSetpoint());

            m_apriltag = target.fiducialId;
            var x_offset = target.bestCameraToTarget.getMeasureX().baseUnitMagnitude();
            var y_offset = target.bestCameraToTarget.getMeasureY().baseUnitMagnitude();
            var rot = target.bestCameraToTarget.getRotation().getAngle();
            if (m_activeX)
                m_desiredChassisSpeeds.vxMetersPerSecond = MathUtil.clamp(-m_xPID.calculate(x_offset), -kMaxAlignmentSpeed, kMaxAlignmentSpeed);
            else
                m_desiredChassisSpeeds.vxMetersPerSecond = 0;
            if (m_activeY)
                m_desiredChassisSpeeds.vyMetersPerSecond = MathUtil.clamp(-m_yPID.calculate(y_offset), -kMaxAlignmentSpeed, kMaxAlignmentSpeed);
            else
                m_desiredChassisSpeeds.vyMetersPerSecond = 0;

            m_desiredChassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(m_rotPID.calculate(rot), -kMaxAlignmentAngularSpeed, kMaxAlignmentAngularSpeed);

            if (m_output) m_drivetrain.driveRobotSpeeds(m_desiredChassisSpeeds);

            SmartDashboard.putNumber("AT_XError", m_xPID.getError());
            SmartDashboard.putNumber("AT_YError", m_yPID.getError());
            SmartDashboard.putNumber("AT_RotError", m_rotPID.getError());
            SmartDashboard.putNumber("AT_DesiredVx", m_desiredChassisSpeeds.vxMetersPerSecond);
            SmartDashboard.putNumber("AT_DesiredVy", m_desiredChassisSpeeds.vyMetersPerSecond);
            SmartDashboard.putNumber("AT_DesiredRot", m_desiredChassisSpeeds.omegaRadiansPerSecond);
            SmartDashboard.putNumber("AT_counter", m_counter);
            m_counter++;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_apriltag = -1;
        m_photonVision.getCamera().takeOutputSnapshot();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if ((!m_activeX || m_xPID.atSetpoint()) && (!m_activeY || m_yPID.atSetpoint()) && m_rotPID.atSetpoint() && m_apriltag > 0)
        {
            System.out.println("ApriltagAlignment isFinished");
            return true;
        }
        return false;
    }
}
