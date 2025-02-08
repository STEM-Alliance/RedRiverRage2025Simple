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
    private final SparkMax m_shooterMotor;

    private double m_elevatorSetpoint = 0.0;
    private final ProfiledPIDController m_elevatorPID = new ProfiledPIDController(
        0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0)
    );

    private double m_shooterSetpoint = 0.0;
    private final ProfiledPIDController m_shooterPID = new ProfiledPIDController(
        0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0)
    );

    private final TofDistanceSubsystem m_tofSensor = new TofDistanceSubsystem(0);

    public ElevatorSubsystem(int elevatorMotorID, int shooterMotorID) {
        m_elevatorMotor = new SparkMax(elevatorMotorID, MotorType.kBrushless);
        m_elevatorMotor.getEncoder().setPosition(0.0);

        m_shooterMotor = new SparkMax(shooterMotorID, MotorType.kBrushless);
        m_shooterMotor.getEncoder().setPosition(0.0);

        m_elevatorPID.setIZone(0.0);
        m_elevatorPID.setIntegratorRange(0.0, 0.0);

        m_shooterPID.setIZone(0.0);
        m_shooterPID.setIntegratorRange(0.0, 0.0);
    }

    public void periodic() {
        elevatorControlLoop();
        shooterControlLoop();

        DataLogHelpers.logDouble(m_elevatorMotor.getOutputCurrent(), "ElevatorSubsystem/Elevator Current");
        DataLogHelpers.logDouble(m_elevatorMotor.getMotorTemperature(), "ElevatorSubsystem/Elevator Temperature");
        DataLogHelpers.logDouble(m_shooterMotor.getOutputCurrent(), "ElevatorSubsystem/Shooter Current");
        DataLogHelpers.logDouble(m_shooterMotor.getMotorTemperature(), "ElevatorSubsystem/Shooter Temperature");
    }

    public final void setElevatorSetpoint(double setpoint) {
        m_elevatorSetpoint = MathUtil.clamp(setpoint, 0.0, 0.0);
        m_elevatorPID.setGoal(m_elevatorSetpoint);
    }

    public final Command setElevatorSetpointCommand(double setpoint) {
        return new InstantCommand(() -> {setElevatorSetpoint(setpoint);});
    }

    private final void elevatorControlLoop() {
        double elevatorOutput = MathUtil.clamp(m_elevatorPID.calculate(0.0), 0.0, 0.0);

        DataLogHelpers.logDouble(elevatorOutput, "ElevatorSubsystem/Elevator PID Output");
        DataLogHelpers.logDouble(m_elevatorSetpoint, "ElevatorSubsystem/Elevator PID Goal");
        DataLogHelpers.logDouble(0.0, "ElevatorSubsystem/Elevator Height");

        m_elevatorMotor.set(elevatorOutput);
    }

    public final void setShooterSetpoint(double setpoint) {
        m_shooterSetpoint = MathUtil.clamp(setpoint, 0.0, 0.0);
        m_shooterPID.setGoal(m_shooterSetpoint);
    }

    public final Command setShooterSetpointCommand(double setpoint) {
        return new InstantCommand(() -> {setShooterSetpoint(setpoint);});
    }

    private final void shooterControlLoop() {
        double shooterOutput = MathUtil.clamp(m_shooterPID.calculate(0.0), 0.0, 0.0);

        DataLogHelpers.logDouble(shooterOutput, "ElevatorSubsystem/Shooter PID Output");
        DataLogHelpers.logDouble(m_shooterSetpoint, "ElevatorSubsystem/Shooter PID Goal");

        m_shooterMotor.set(shooterOutput);
    }

    private final boolean shooterIsLoaded() {
        int tofDistance = m_tofSensor.get_distance();
        int tofStatus = m_tofSensor.get_status();

        return (tofStatus == 0) && (tofDistance < 75);
    }
}
