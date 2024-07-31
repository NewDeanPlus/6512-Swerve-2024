package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.*;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.sensors.ThriftyEncoder;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    //public final CANCoder absoluteEncoder;
    public final ThriftyEncoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad; 

    private final String moduleName;

    private double pidCalculations;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, String moduleName) {
        
        this.moduleName = moduleName;
                
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        //absoluteEncoder = new CANCoder(absoluteEncoderId);
        absoluteEncoder = new ThriftyEncoder(absoluteEncoderId);

        // driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        // driveMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.setIdleMode(IdleMode.kBrake);
        //Used to be kBrake

        turningMotor.setIdleMode(IdleMode.kBrake);
        //Used to be kCoast
        

        // Sets the ramp rate for the drive and turning motors
        // Controls how fast you can accelerate when using onboard PID (Not currently used in tuning)
        driveMotor.setClosedLoopRampRate(0.5); //0.15
        turningMotor.setClosedLoopRampRate(0.15); //0.08


        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 1, 0.001);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        // absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
//TODO - limit absolute encoder range to -180 to 180

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        // Use the CANCoder for turning position
        return getAbsoluteEncoderRad();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getPosition();
        // angle *= Math.PI/180.0;
        angle -= absoluteEncoderOffsetRad;
        angle = (angle * (absoluteEncoderReversed ? -1.0 : 1.0)) % (2*Math.PI);
        if(angle > Math.PI) {
            angle -= (2*Math.PI);
        }
        else if(angle < (-1*Math.PI)) {
            angle += (2*Math.PI);
        }

        return angle;
    }

    public double getRawEncoderValue() {
        double angle = absoluteEncoder.getPosition();
        // angle *= Math.PI/180.0;
        return (angle * (absoluteEncoderReversed ? -1.0 : 1.0) % (2*Math.PI));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
        getDrivePosition() , new Rotation2d(getTurningPosition()));

    }   
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        pidCalculations = turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians());
        //System.out.println(pidCalculations);

        turningMotor.set(pidCalculations);
        SmartDashboard.putString("Swerve[" + moduleName + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
