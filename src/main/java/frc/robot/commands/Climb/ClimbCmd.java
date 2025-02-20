package frc.robot.commands.Climb;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.climbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCmd extends Command {
    ClimbSubsystem climbSubsystem;
    double climbSpeed, clawSpeed;
    private final Supplier<Boolean> climbUpButton, climbDownButton;
    private final Supplier<Boolean> clawOpenButton, clawCloseButton;

    public ClimbCmd(ClimbSubsystem climbSubsystem, Supplier<Boolean> climbUpButton,
        Supplier<Boolean> climbDownButton, Supplier<Boolean> clawOpenButton, Supplier<Boolean> clawCloseButton){
        
        this.climbSubsystem = climbSubsystem;
        this.climbUpButton = climbUpButton;
        this.climbDownButton = climbDownButton;
        this.clawOpenButton = clawOpenButton;
        this.clawCloseButton = clawCloseButton;
        
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(clawOpenButton.get()){climbSubsystem.incrementClawSetpoint(0.02);}
        if(clawCloseButton.get()){climbSubsystem.incrementClawSetpoint(-0.02);}

        if(climbUpButton.get()){climbSubsystem.incrementClimbSetpoint(1);}
        if(climbDownButton.get()){climbSubsystem.incrementClimbSetpoint(-1);}

        climbConstants.climbSetpoint = Math.min(Math.max(climbConstants.climbSetpoint, climbConstants.climbMin), climbConstants.climbMax);
        climbSubsystem.setPlanetary(climbConstants.climbSetpoint);

        climbConstants.clawSetpoint = Math.min(Math.max(climbConstants.clawSetpoint, climbConstants.clawMin), climbConstants.clawMax);
        climbSubsystem.setClaw(climbConstants.clawSetpoint);
    }

    @Override
    public void end(boolean interrupted){
    }

}
