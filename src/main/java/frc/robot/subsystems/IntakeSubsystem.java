package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(intakeConstants.intakeMotorID, MotorType.kBrushless);
    private final SparkMax controlMotor = new SparkMax(intakeConstants.controlMotorID, MotorType.kBrushless);
    private final SparkMax hoodMotor = new SparkMax(intakeConstants.hoodMotorID, MotorType.kBrushless);
    private final CANrange CANrangeFirst = new CANrange(intakeConstants.CANrangeFirstID);
    private final CANrange CANrangeSecond = new CANrange(intakeConstants.CANrangeSecondID);
    private final CANrangeConfiguration CANrangeConfig = new CANrangeConfiguration();
    private final SparkMaxConfig intakeMotorConfig;
    private double distanceFirst, distanceSecond;

    public IntakeSubsystem (){
        intakeMotorConfig = new SparkMaxConfig();
        intakeMotorConfig.idleMode(IdleMode.kBrake);
        intakeMotorConfig.inverted(true);

        CANrangeFirst.getConfigurator().apply(CANrangeConfig);
        CANrangeSecond.getConfigurator().apply(CANrangeConfig);

        intakeMotor.configure(intakeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);     
    }

    public double getPosition(){
        return intakeMotor.getEncoder().getPosition();
    }

    public boolean getIsFirstDetected(){
        distanceFirst = CANrangeFirst.getDistance(true).getValueAsDouble();
        return distanceFirst < intakeConstants.kCANrangeDetectionLimit;
    }

    public boolean getIsSecondDetected(){
        distanceSecond = CANrangeSecond.getDistance(true).getValueAsDouble();
        return distanceSecond < intakeConstants.kCANrangeDetectionLimit;
    }

    public void setStopMode(){
        intakeMotor.set(0);
    }

    public void setPlanetary(double speed){
        speed = Math.min(Math.max(speed, -0.2), intakeConstants.intakeMaxSpeed);

        intakeMotor.set(speed);
    }

    public void setIntake(double speed){
        controlMotor.set(speed);
    }

    public void setHood(double speed){
        hoodMotor.set(speed);
    }

    public void changeSetpoint(double setpoint){
        intakeConstants.intakeSetpoint = setpoint;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed", controlMotor.getEncoder().getVelocity());
    }

}
