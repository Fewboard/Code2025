package frc.robot.commands.Autos;

import java.util.function.Supplier;

// ======================= IMPORTAÇÃO DE BIBLIOTECAS =======================
    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.Constants.elevatorConstants;
    import frc.robot.Constants.intakeConstants;
    import frc.robot.subsystems.ElevatorSubsystem;
    import frc.robot.subsystems.IntakeSubsystem;
// ============================================================================

public class CollectCmd extends Command {
    // =================== INSTANCIA OS SUBSISTEMAS E VARIAVEIS =================
    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final Supplier<Boolean> cancelButton;
    // ============================================================================

    public CollectCmd(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, 
                Supplier<Boolean> cancelButton) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.cancelButton = cancelButton;

    }

    @Override
    public void initialize() {
        elevatorSubsystem.changeSetpoint(elevatorConstants.CollectPosition);
        intakeSubsystem.changeSetpoint(intakeConstants.CollectPosition);

        intakeSubsystem.setHood(-0.15);
        intakeSubsystem.setIntake(0.25);
        intakeConstants.intakeCollecting = true;
        //System.out.println("Inicialized");
    }

    @Override
    public void execute() {
        if (intakeSubsystem.getIsSecondDetected()){
            intakeSubsystem.setIntake(0.12);
            //System.out.println("Second Detected");
        }
        //System.out.println("Execute");
    }

    @Override
    public void end(boolean interrupted) {
        //System.out.println("Ended");
        intakeConstants.intakeCollecting = false;
        intakeSubsystem.setIntake(0);
        intakeSubsystem.setHood(0);
    }

    @Override
    public boolean isFinished() {
        System.out.println("Botão: " + cancelButton.get());
        return (!intakeSubsystem.getIsFirstDetected() && intakeSubsystem.getIsSecondDetected()) || cancelButton.get();
    }
}
