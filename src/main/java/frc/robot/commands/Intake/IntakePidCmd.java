package frc.robot.commands.Intake;

import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePidCmd extends Command {
    private final Supplier<Integer> pov;
    private final Supplier<Boolean> intakeInputButton, intakeOutputButton;
    private final Supplier<Boolean> hoodInputButton, hoodOutputButton;
    private double error;

    IntakeSubsystem intakeSubsystem;
    PIDController intakePidController = new PIDController(0.05,0,0.0000001);

    public IntakePidCmd(IntakeSubsystem intakeSubsystem, Supplier<Integer> pov, Supplier<Boolean> intakeInputButton,
                Supplier<Boolean> intakeOutputButton, Supplier<Boolean> hoodInputButton, Supplier<Boolean> hoodOutputButton){
        this.intakeSubsystem = intakeSubsystem;
        this.pov = pov;
        this.intakeInputButton = intakeInputButton;
        this.intakeOutputButton = intakeOutputButton;
        this.hoodInputButton = hoodInputButton;
        this.hoodOutputButton = hoodOutputButton;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        if(pov.get() == OIConstants.kRaiseIntakeButtonIdx){intakeSubsystem.changeSetpoint(intakeConstants.intakeSetpoint + 0.2);} 
        if(pov.get() == OIConstants.kLowerIntakeButtonIdx){intakeSubsystem.changeSetpoint(intakeConstants.intakeSetpoint - 0.2);}

        if (intakeInputButton.get()) {
            intakeSubsystem.setIntake(0.3);
        } else if (intakeOutputButton.get()) {
            intakeSubsystem.setIntake(-0.3);
        }

        if (hoodInputButton.get()) {
            intakeSubsystem.setHood(0.2);
        } else if (hoodOutputButton.get()) {
            intakeSubsystem.setHood(-0.2);
        }

        intakeConstants.intakeSetpoint = Math.min(Math.max(intakeConstants.intakeSetpoint, intakeConstants.intakeMin), intakeConstants.intakeMax);
        error = intakePidController.calculate(intakeSubsystem.getPosition(), intakeConstants.intakeSetpoint);
        
        intakeSubsystem.setPlanetary(error);
    }

    @Override
    public void end(boolean interrupted){
    }

}
