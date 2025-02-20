package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climbConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbSubsystem extends SubsystemBase {
    private SparkClosedLoopController pidClimbController;
    private SparkClosedLoopController pidClawController;

    private final SparkMax planetaryMotor = new SparkMax(climbConstants.planetaryMotorID, MotorType.kBrushless);
    private final SparkMaxConfig planetaryMotorConfig = new SparkMaxConfig();
    private final SparkMax clawMotor = new SparkMax(climbConstants.clawMotorID, MotorType.kBrushless);
    private final SparkMaxConfig clawMotorConfig = new SparkMaxConfig();

    public ClimbSubsystem (){
        pidClimbController = planetaryMotor.getClosedLoopController();
        planetaryMotorConfig.closedLoop.p(climbConstants.climbP);
        planetaryMotorConfig.idleMode(IdleMode.kBrake);
        planetaryMotorConfig.inverted(true);
        planetaryMotor.configure(planetaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
        
        pidClawController = clawMotor.getClosedLoopController();
        clawMotorConfig.closedLoop.p(climbConstants.clawP);
        clawMotorConfig.idleMode(IdleMode.kBrake);
        
        clawMotor.configure(clawMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    
    }

    public double getClimbPosition(){
        return planetaryMotor.getEncoder().getPosition();
    }

    public double getClawPosition(){
        return clawMotor.getEncoder().getPosition();
    }

    public void setClimbStopMode(){
        planetaryMotor.set(0);
    }

    public void setClawStopMode(){
        clawMotor.set(0);
    }

    public void setPlanetary(double position){
        pidClimbController.setReference(position, ControlType.kPosition);
    }

    public void setClaw(double position){
        pidClawController.setReference(position, ControlType.kPosition);
    }

    public double getClimbSetpoint(){
        return climbConstants.climbSetpoint;
    }

    public void changeClimbSetpoint(double setpoint){
        climbConstants.climbSetpoint = setpoint;
    }

    public void incrementClimbSetpoint(double setpoint){
        climbConstants.climbSetpoint += setpoint;
    }
    
    public double getClawSetpoint(){
        return climbConstants.clawSetpoint;
    }

    public void changeClawSetpoint(double setpoint){
        climbConstants.clawSetpoint = setpoint;
    }
    
    public void incrementClawSetpoint(double setpoint){
        climbConstants.clawSetpoint += setpoint;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Position", planetaryMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Climb Speed", planetaryMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Climb Setpoint", getClimbSetpoint());

        SmartDashboard.putNumber("Claw Position", clawMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Claw Speed", clawMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Claw Setpoint", getClawSetpoint());
    }

}
