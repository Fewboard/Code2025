package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotor = new SparkMax(elevatorConstants.elevatorMotorID, MotorType.kBrushless);
    private final SparkMaxConfig elevatorMotorConfig;

    public ElevatorSubsystem (){
        //elevatorMotorConfig.absoluteEncoder.positionConversionFactor(0.05);
        //elevatorMotor.configure(elevatorMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters)
        //absoluteEncoder = elevatorMotor.getAbsoluteEncoder();

        elevatorMotorConfig = new SparkMaxConfig();
        elevatorMotorConfig.idleMode(IdleMode.kBrake);
        elevatorMotorConfig.inverted(true);

        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);     
    }

    public double getPosition(){
        return elevatorMotor.getEncoder().getPosition();
    }

    public void setStopMode(){
        elevatorMotor.set(0);
    }

    public void setMotor(double speed){
        SmartDashboard.putNumber("Elevator Speed", speed);

        elevatorMotor.set(speed);
    }

    public void changeSetpoint(double setpoint){
        elevatorConstants.elevatorSetpoint = setpoint;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevador", getPosition());
    }

}
