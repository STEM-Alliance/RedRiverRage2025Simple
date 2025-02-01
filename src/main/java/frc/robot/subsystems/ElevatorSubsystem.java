package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.DataLogHelpers;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_elevatorMotor;

    private double m_setpoint = 0.0;
    private final ProfiledPIDController m_elevatorPID = new ProfiledPIDController(
        0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0)
    );

    public ElevatorSubsystem(int motorID) {
        m_elevatorMotor = new SparkMax(motorID, MotorType.kBrushless);
        m_elevatorMotor.getEncoder().setPosition(0.0);

        m_elevatorPID.setIZone(0.0);
        m_elevatorPID.setIntegratorRange(0.0, 0.0);
    }

    public void periodic() {
        elevatorControlLoop();

        DataLogHelpers.logDouble(m_elevatorMotor.getOutputCurrent(), "ElevatorSubsystem/Elevator Current");
    }

    public final void setSetpoint(double setpoint) {
        m_setpoint = setpoint;
    }

    public final Command setSetpointCommand(double setpoint) {
        return new InstantCommand(() -> {setSetpoint(setpoint);});
    }

    private final void elevatorControlLoop() {
        m_setpoint = MathUtil.clamp(m_setpoint, 0.0, 0.0);
        m_elevatorPID.setGoal(m_setpoint);

        double elevatorOutput = MathUtil.clamp(m_elevatorPID.calculate(0.0), 0.0, 0.0);

        DataLogHelpers.logDouble(elevatorOutput, "ElevatorSubsystem/PID Output");
        DataLogHelpers.logDouble(m_setpoint, "ElevatorSubsystem/PID Goal");
        DataLogHelpers.logDouble(0.0, "ElevatorSubsystem/Elevator Height");

        m_elevatorMotor.set(elevatorOutput);
    }
}
