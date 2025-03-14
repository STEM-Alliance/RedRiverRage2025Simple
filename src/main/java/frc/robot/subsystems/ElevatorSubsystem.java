package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.kShooterKi;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.kElevatorSetpoints;
import frc.robot.Constants.kShooterSetpoints;
import frc.robot.util.DataLogHelpers;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_elevatorMotor;
    private final SparkMax m_shooterMotor;
    private final RelativeEncoder m_elevatorEncoder;
    private final DigitalInput m_limitHigh = new DigitalInput(0);
    private final DigitalInput m_limitLow = new DigitalInput(3);
    private final AbsoluteEncoder m_intakePos;

    private double m_elevatorSetpoint = 0.0;
    private final ProfiledPIDController m_elevatorPID = new ProfiledPIDController(
        Constants.kElevatorKp, 0.0, 0.0, new TrapezoidProfile.Constraints(Constants.kElevatorMaxVelocity, Constants.kElevatorMaxAcceleration)
    );

    private double m_shooterSetpoint = 0.0;
    private final ProfiledPIDController m_shooterPID = new ProfiledPIDController(
        Constants.kShooterKp, Constants.kShooterKi, 0.0, new TrapezoidProfile.Constraints(Constants.kShooterMaxVelocity, Constants.kShooterMaxVelocity)
    );

    //private final IntakeSubsystem m_intake = new IntakeSubsystem(11, 0);
    // dio 1 and 3

    public ElevatorSubsystem(int elevatorMotorID, int shooterMotorID) {
        m_elevatorMotor = new SparkMax(elevatorMotorID, MotorType.kBrushless);
        m_elevatorEncoder = m_elevatorMotor.getEncoder();
        m_elevatorEncoder.setPosition(0.0);

        m_shooterMotor = new SparkMax(shooterMotorID, MotorType.kBrushed);
        m_shooterMotor.getEncoder().setPosition(0.0);

        m_elevatorPID.setIZone(0.0);
        m_elevatorPID.setIntegratorRange(0.0, 0.0);

        m_shooterPID.setIZone(0.15);
        m_shooterPID.setIntegratorRange(-0.5, 0.5);

        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        SparkMaxConfig shooterConfig = new SparkMaxConfig();

        elevatorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.NeoLimit);
        // I tried to call the invert function and change the factor to -1. In both cases it crashes
        elevatorConfig.encoder
            // Magic numbers found through trial and error; ask Joe why our CAD isnt accurate.
            // (output is in inches, inches per second(?))
            .positionConversionFactor((1.5 / 0.54969295458888695567845703186279 * Math.PI * (25.0 / 12.0)) / 25.0)
            .velocityConversionFactor((1.5 / 0.54969295458888695567845703186279 * Math.PI * (25.0 / 12.0)) / 25.0 / 60.0);
        
        shooterConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.WindowLimit);
        shooterConfig.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);
        shooterConfig.limitSwitch.
            forwardLimitSwitchEnabled(false).
            reverseLimitSwitchEnabled(false);

        m_elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_intakePos = m_shooterMotor.getAbsoluteEncoder();
        setShooterSetpoint(kShooterSetpoints.IDLE.getAsDouble());

        m_elevatorPID.setTolerance(0.5);
        m_shooterPID.setTolerance(0.025);
        m_shooterPID.enableContinuousInput(0.0, 1.0);
        setDefaultCommand(getControlLoopCommand());
    }

    public void periodic() {
        SmartDashboard.putNumber("ElevatorSubsystem/Elevator Current", m_elevatorMotor.getOutputCurrent());
        DataLogHelpers.logDouble(m_elevatorMotor.getMotorTemperature(), "ElevatorSubsystem/Elevator Temperature");
        SmartDashboard.putNumber("ElevatorSubsystem/Shooter Current", m_shooterMotor.getOutputCurrent());
        DataLogHelpers.logDouble(m_shooterMotor.getMotorTemperature(), "ElevatorSubsystem/Shooter Temperature");
        SmartDashboard.putNumber("ElevatorPos", -m_elevatorEncoder.getPosition());
        SmartDashboard.putBoolean("LimitLow", m_limitLow.get());
        SmartDashboard.putBoolean("LimitHigh", m_limitHigh.get());
        SmartDashboard.putNumber("IntakePos", m_intakePos.getPosition());
    }

    public final Command getControlLoopCommand() {
        return new RunCommand(
            () -> {elevatorControlLoop(); shooterControlLoop();},
            this
        );
    }

    public final void setElevatorSetpoint(double setpoint) {
        m_elevatorSetpoint = MathUtil.clamp(setpoint, 0.0, kElevatorSetpoints.L4.getAsDouble());
        m_elevatorPID.setGoal(m_elevatorSetpoint);
    }

    public final Command setElevatorSetpointCommand(double setpoint) {
        return new InstantCommand(() -> {setElevatorSetpoint(setpoint);});
    }

    private final void elevatorControlLoop() {
        SmartDashboard.putData("ElevatorPID", m_elevatorPID);
        double elevatorOutput = MathUtil.clamp(m_elevatorPID.calculate(m_elevatorEncoder.getPosition()), -0.6, 0.75);
        SmartDashboard.putNumber("elevatorOutput", elevatorOutput);

        DataLogHelpers.logDouble(elevatorOutput, "ElevatorSubsystem/Elevator PID Output");
        DataLogHelpers.logDouble(m_elevatorSetpoint, "ElevatorSubsystem/Elevator PID Goal");
        DataLogHelpers.logDouble(0.0, "ElevatorSubsystem/Elevator Height");
        SmartDashboard.putNumber("Elevator Position", m_elevatorEncoder.getPosition());
        SmartDashboard.putNumber("Error", m_elevatorPID.getPositionError());

        if ((elevatorOutput < 0.0) && (m_limitLow.get())) {
            m_elevatorMotor.set(elevatorOutput);
        }

        else if ((elevatorOutput > 0.0) && (m_limitHigh.get())) {
            m_elevatorMotor.set(elevatorOutput);
        }

        else {
            m_elevatorMotor.set(0.0);
        }
    }

    public final void setShooterSetpoint(double setpoint) {
        m_shooterSetpoint = MathUtil.clamp(setpoint, -1, 1);
        m_shooterPID.setGoal(m_shooterSetpoint);
    }

    public final Command setShooterSetpointCommand(double setpoint) {
        return new InstantCommand(() -> {setShooterSetpoint(setpoint);});
    }

    private final void shooterControlLoop() {
        SmartDashboard.putData("ShooterPID", m_shooterPID);
        double shooterOutput = MathUtil.clamp(-m_shooterPID.calculate(m_intakePos.getPosition()), -0.8, 0.8);
        SmartDashboard.putNumber("ShooterOutput", shooterOutput);

        DataLogHelpers.logDouble(shooterOutput, "ElevatorSubsystem/Shooter PID Output");
        DataLogHelpers.logDouble(m_shooterSetpoint, "ElevatorSubsystem/Shooter PID Goal");

        m_shooterMotor.set(shooterOutput);
    }

    public final Command setState(
        kElevatorSetpoints elevatorSetpoint,
        kShooterSetpoints shooterSetpoint
    ) {
        return new InstantCommand(() -> {
            if (elevatorSetpoint != null) setElevatorSetpoint(elevatorSetpoint.getAsDouble());
            if (shooterSetpoint != null) setShooterSetpoint(shooterSetpoint.getAsDouble());
        });
    }

    public final Command setStateIdle() {
        return setState(kElevatorSetpoints.IDLE, kShooterSetpoints.IDLE);
    }

    public final Command zeroHeight() {
        return new FunctionalCommand(
            () -> {},
            () -> {m_elevatorMotor.set(-0.10); m_elevatorEncoder.setPosition(0.0);},
            interrupted -> stop(),
            () -> !m_limitLow.get(),
            this
        );
    }

    public final Command emergencyStop() {
        return new InstantCommand(() -> m_elevatorMotor.set(0.0), this);
    }

    public final Boolean stop()
    {
        m_elevatorMotor.set(0);
        m_elevatorEncoder.setPosition(0.0);
        return true;
    }

    public final Command up() {
        return new FunctionalCommand(() -> {}, 
                                     ()->{m_elevatorMotor.set(1);},
                                     interrupted -> stop(),
                                     ()->false);
    }

    public final Command down() {
        return new FunctionalCommand(() -> {}, 
                                     ()->{m_elevatorMotor.set(-1);},
                                     interrupted -> stop(),
                                     ()->false);
    }

    public final Boolean stopRotate()
    {
        m_shooterMotor.set(0);
        return true;
    }

    public final Command cw() {
        return new FunctionalCommand(() -> {}, 
                                     ()->{m_shooterMotor.set(0.5);},
                                     interrupted -> stopRotate(),
                                     ()->false);
    }

    public final Command ccw() {
        return new FunctionalCommand(() -> {}, 
                                     ()->{m_shooterMotor.set(-0.5);},
                                     interrupted -> stopRotate(),
                                     ()->false);
    }

    public final Command atSetpoints() {
        return new FunctionalCommand(
            () -> {},
            () -> {},
            interrupted -> {},
            () -> (m_elevatorPID.atGoal() && m_shooterPID.atGoal())
        );
    }

    public final Command interruptControl() {
        return new RunCommand(
            () -> {m_elevatorMotor.set(0.0);},
            this
        );
    }

    // public final Command startIntaking() {
    //     return new InstantCommand(() -> {
    //         setState(null, kShooterSetpoints.INTAKE);
    //     }).alongWith(m_intake.startIntaking());
    // }

    // public final Command stopIntaking() {
    //     return new InstantCommand(() -> {
    //         setState(null, kShooterSetpoints.IDLE);
    //     }).alongWith(m_intake.stopIntaking());
    // }

    // public final Command startShooting() {
    //     return new InstantCommand(() -> {
    //         setState(null, kShooterSetpoints.SHOOT);
    //     }).alongWith(m_intake.startShooting());
    // }

    // public final Command stopShooting() {
    //     return new InstantCommand(() -> {
    //         setState(null, kShooterSetpoints.IDLE);
    //     }).alongWith(m_intake.stopShooting());
    // }
}
