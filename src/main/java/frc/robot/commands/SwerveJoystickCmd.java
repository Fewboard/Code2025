package frc.robot.commands;

// ======================= IMPORTAÇÃO DE BIBLIOTECAS =======================
    import edu.wpi.first.math.filter.SlewRateLimiter;
    import edu.wpi.first.math.kinematics.ChassisSpeeds;
    import edu.wpi.first.math.kinematics.SwerveModuleState;
    import edu.wpi.first.wpilibj2.command.Command;
    import java.util.function.Supplier;
    import frc.robot.Constants.DriveConstants;
    import frc.robot.Constants.OIConstants;
    import frc.robot.subsystems.LimelightSubsystem;
    import frc.robot.subsystems.SwerveSubsystem;
// ============================================================================

public class SwerveJoystickCmd extends Command {
    
    // =================== INSTANCIA OS SUBSISTEMAS E VARIAVEIS =================
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> resetEncoderButton;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    // ============================================================================

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem, Supplier<Double> xSpdFunction, 
                Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> resetEncoderButton) {
        
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.resetEncoderButton = resetEncoderButton;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (resetEncoderButton.get()){
            swerveSubsystem.resetSwerve();
        }

        // 1. Coleta a posição do Joystick
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Aplica uma tolerância
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Otimiza a velocidade
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 5. Calcula a velocidade do robô, em relação a arena
        ChassisSpeeds chassisSpeeds;
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

        // 6. Calcula a velocidade individual de cada módulo Swerve
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 7. Move os módulos para a posição e aceleração desejadas
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
