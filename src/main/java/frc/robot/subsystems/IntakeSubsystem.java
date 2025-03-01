package frc.robot.subsystems;

import java.time.Period;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax m_intakeMotor;
    private final TofDistanceSubsystem m_tofSensor;
    private boolean m_previousCoralDetected = false;
    private boolean m_coralDetected = false;
    private int m_rumbleCounter = 0;
    private final CommandXboxController m_controller;

    private final Debouncer m_tofSensorDebouncer = new Debouncer(0.02 * 3);

    public IntakeSubsystem(int intakeMotorID, int tofSensorID, CommandXboxController controller) {
        m_intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);
        m_tofSensor = new TofDistanceSubsystem(tofSensorID);
        m_controller = controller;

        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        intakeConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(Constants.Neo550Limit);

        m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic() {
        if (m_coralDetected & !m_previousCoralDetected)
        {
            m_rumbleCounter = Constants.kRumbleTimer;
        }
        m_previousCoralDetected = m_coralDetected;
        if (m_rumbleCounter > 0)
        {
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
            m_rumbleCounter--;
        }
        else 
        {
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 0);
        }
    }

    public final boolean checkAndStopIntake()
    {
        //if (m_tofSensorDebouncer.calculate(isIntakeLoaded()))
        if (isIntakeLoaded())
        {
            m_intakeMotor.set(0);
            return true;
        }
        return false;
    }

    public final Command startIntaking() {
        return new FunctionalCommand(
            () -> {m_intakeMotor.set(1.0); System.out.println("Set 1.0");},
            () -> {System.out.println("Intaking...");},
            interrupted -> {m_intakeMotor.set(0.0);},
            () -> checkAndStopIntake(),
            this
        ).andThen(Commands.deadline(
            new WaitCommand(0.175),
            new InstantCommand(() -> m_intakeMotor.set(-1.0))
        )).andThen(
            new InstantCommand(() -> m_intakeMotor.set(0.0))
        );
    }

    public final Command stopIntaking() {
        return new InstantCommand(
            () -> {m_intakeMotor.set(0.0); System.out.println("Set 0.0");},
            this
        );
    }

    public final Command startShooting() {
        return new InstantCommand(
            () -> {m_intakeMotor.set(1.0); System.out.println("Set 1.0");},
            this
        );
    }

    public final Command stopShooting() {
        return new InstantCommand(
            () -> {m_intakeMotor.set(0.0); System.out.println("Set 0.0");},
            this
        );
    }

    // public final Command autonomousShoot() {
    //     return new FunctionalCommand(
    //         () -> m_intakeMotor.set(1.0),
    //         () -> {},
    //         interrupted -> {},
    //         () -> isIntakeNotLoaded(),
    //         this
    //     ).until
    // }

    public final boolean isIntakeLoaded() {
        if (m_tofSensor.is_within_threshold(75))
        {
            m_coralDetected = true;
        }
        else
        {
            m_coralDetected = false;
        }
        return m_coralDetected;
    }

    public final boolean isIntakeNotLoaded() {
        return !isIntakeLoaded();
    }
}
