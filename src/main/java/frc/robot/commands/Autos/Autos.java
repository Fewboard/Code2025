package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.elevatorConstants;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Autos {
    public static Command Shoot(IntakeSubsystem intakeSubsystem){
        return Commands.sequence(
            new InstantCommand(()-> intakeSubsystem.setIntake(intakeSubsystem.getPosition() < 10 ? 0.3 : -0.3)),
            new WaitCommand(1),
            new InstantCommand(()-> intakeSubsystem.setIntake(0))
            
            );
    }

    public static Command L1Position(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem){
        return Commands.sequence(
            new WaitUntilCommand(()-> !intakeConstants.intakeCollecting),
            new InstantCommand(()-> elevatorSubsystem.changeSetpoint(elevatorConstants.L1Position)),
            new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.L1Position))
            );
    }

    public static Command L2Position(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem){
        return Commands.sequence(
            new WaitUntilCommand(()-> !intakeConstants.intakeCollecting),
            new InstantCommand(()-> elevatorSubsystem.changeSetpoint(elevatorConstants.L2Position)),
            new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.L2Position))
            );
    }

    public static Command L3Position(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem){
        return Commands.sequence(
            new WaitUntilCommand(()-> !intakeConstants.intakeCollecting),
            new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.L3Position)),
            new InstantCommand(()-> elevatorSubsystem.changeSetpoint(elevatorConstants.L3Position))
            );
    }

    public static Command L4Position(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem){
        return Commands.sequence(
            new WaitUntilCommand(()-> !intakeConstants.intakeCollecting),
            new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.L4Position)),
            new InstantCommand(()-> elevatorSubsystem.changeSetpoint(elevatorConstants.L4Position))
            );
    }
    
}
