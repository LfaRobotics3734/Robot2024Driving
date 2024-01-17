package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class MAXswerve {
    CANSparkMax drivingMotor;
    CANSparkMax turningMotor;

    RelativeEncoder drivingEncoder;
    AbsoluteEncoder turningEncoder;

    SparkMaxPIDController drivingPID;
    SparkMaxPIDController turningPID;

    double angleOffset;
    SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public MAXswerve(int DrivingMotorCanID, int turningMOtorCanID, double angleOffset){
        drivingMotor = new CANSparkMax(DrivingMotorCanID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMOtorCanID, MotorType.kBrushless);
        this.angleOffset = angleOffset;

        drivingEncoder = drivingMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
        drivingPID = drivingMotor.getPIDController();
        turningPID = turningMotor.getPIDController();
        drivingPID.setFeedbackDevice(drivingEncoder);
        turningPID.setFeedbackDevice(turningEncoder);

        drivingEncoder.setPositionConversionFactor(Constants.Drive.positionConversionFactor);
        drivingEncoder.setVelocityConversionFactor(Constants.Drive.velocityConversionFactor);
        turningEncoder.setPositionConversionFactor(Constants.Turn.positionConversionFactor);
        turningEncoder.setVelocityConversionFactor(Constants.Turn.velocityConversionFactor);

        turningPID.setPositionPIDWrappingEnabled(true);
        turningPID.setPositionPIDWrappingMinInput(0);
        turningPID.setPositionPIDWrappingMaxInput(2 * Math.PI); 

        drivingPID.setP(Constants.Drive.p);
        drivingPID.setI(Constants.Drive.i);
        drivingPID.setD(Constants.Drive.d);
        drivingPID.setFF(Constants.Drive.ff);
        drivingPID.setOutputRange(-1, 1);

        turningPID.setP(Constants.Turn.p);
        turningPID.setI(Constants.Turn.i);
        turningPID.setD(Constants.Turn.d);
        turningPID.setFF(Constants.Turn.ff);
        turningPID.setOutputRange(-1, 1);

        drivingMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);

        drivingMotor.burnFlash();
        turningMotor.burnFlash();
        
        drivingEncoder.setPosition(0);
    }

    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
            drivingEncoder.getPosition(),
            new Rotation2d(turningEncoder.getPosition() - angleOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(angleOffset));
    
        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
            new Rotation2d(turningEncoder.getPosition()));

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        drivingPID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turningPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    
        this.desiredState = desiredState;
    }
    public boolean isMoving(){
return desiredState.speedMetersPerSecond!=0;  
}
    public void resetEncoders() {
        drivingEncoder.setPosition(0);
    }

    public void setP(double P){
        drivingPID.setP(P);
    }

    public void setForward(){
        turningPID.setReference(0 + angleOffset, ControlType.kPosition);
    }

}
