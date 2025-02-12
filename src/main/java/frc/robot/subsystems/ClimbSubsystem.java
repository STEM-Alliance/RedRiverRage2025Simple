package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    public ClimbSubsystem() {}

    public final Command toggleClimber() {
        return new InstantCommand();
    }

    public final Command toggleClaw() {
        return new InstantCommand();
    }
}
