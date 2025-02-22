package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax m_intakeMotor;
    private final TofDistanceSubsystem m_tofSensor;

    private final Debouncer m_tofSensorDebouncer = new Debouncer(0.02 * 3);

    public IntakeSubsystem(int intakeMotorID, int tofSensorID) {
        m_intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);
        m_tofSensor = new TofDistanceSubsystem(tofSensorID);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        intakeConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(Constants.Neo550Limit);

        m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public final Command startIntaking() {
        return new FunctionalCommand(
            () -> {m_intakeMotor.set(1.0); System.out.println("Set 1.0");},
            () -> {System.out.println("Intaking...");},
            interrupted -> {m_intakeMotor.set(0.0);},
            () -> m_tofSensorDebouncer.calculate(isIntakeLoaded()),
            this
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

    public final boolean isIntakeLoaded() {
        return m_tofSensor.is_within_threshold(75, false);
    }
}
