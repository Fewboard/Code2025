package frc.robot.commands.LedStrip;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedStripSubsystem;

public class LEDControlCmd extends Command {

    private LedStripSubsystem ledSubsystem;
    
    public LEDControlCmd(LedStripSubsystem ledSubsystem){
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        System.out.println("Ligada");
        ledSubsystem.ledStrip(0, 106, 0, 0, 255);
    }

    @Override
    public void end(boolean interrupted){
    }

}
