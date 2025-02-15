// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class ClimbSubsystem extends SubsystemBase {
//     private final Solenoid m_climberSolenoid;
//     private final Solenoid m_clawSolenoid;
    
//     public ClimbSubsystem(int climberSolenoidID, int clawSolenoidID) {
//         m_climberSolenoid = new Solenoid(PneumaticsModuleType.REVPH, climberSolenoidID);
//         m_clawSolenoid = new Solenoid(PneumaticsModuleType.REVPH, clawSolenoidID);
//     }

//     public final Command toggleClimber() {
//         return new InstantCommand(() -> {
//             m_climberSolenoid.toggle();
//         });
//     }

//     public final Command toggleClaw() {
//         return new InstantCommand(() -> {
//             m_clawSolenoid.toggle();
//         });
//     }
// }
