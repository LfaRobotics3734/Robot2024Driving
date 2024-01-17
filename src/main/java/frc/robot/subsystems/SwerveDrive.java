package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

//import edu.wpi.first.apriltag.AprilTagFieldLayout;
//import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;

public class SwerveDrive extends SubsystemBase {
    //motors
    MAXswerve frontLeft = new MAXswerve(Constants.IO.frontLeftDrive, Constants.IO.frontLeftTurn, Constants.Turn.angleOffsets[1]);
    MAXswerve frontRight = new MAXswerve(Constants.IO.frontRightDrive, Constants.IO.frontRightTurn, Constants.Turn.angleOffsets[0]);
    MAXswerve backLeft = new MAXswerve(Constants.IO.backLeftDrive, Constants.IO.backLeftTurn, Constants.Turn.angleOffsets[3]);
    MAXswerve backRight = new MAXswerve(Constants.IO.backRightDrive, Constants.IO.backRightTurn, Constants.Turn.angleOffsets[2]);

    //gyrometer
    private final AHRS gyro = new AHRS();

    //slew rate motor
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    // balance variables
    private double balanceDirection = Double.POSITIVE_INFINITY;
    private boolean balancing = true;
    private boolean over = false;
    private double avgAngle = 5.0;


    //gear
    private double ratio = 1;
    // Limelight

    // private int counter = 0;


    // Odometry class for tracking robot pose
    // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    //     DriveConstants.kDriveKinematics,
    //     Rotation2d.fromDegrees(-gyro.getYaw()),
    //     new SwerveModulePosition[] {
    //         frontLeft.getPosition(),
    //         frontRight.getPosition(),
    //         backLeft.getPosition(),
    //         backRight.getPosition()
    // });

    // Same as above with ability to consider vision measurements :)
    private SwerveDrivePoseEstimator m_poseEstimator;

    
    //initiate swerve drive object
    
    @Override
    public void periodic() {
       
        //update the pose estimator
        m_poseEstimator.update(
            Rotation2d.fromDegrees(-gyro.getYaw()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );

        //System.out.println("Position: " + m_poseEstimator.getEstimatedPosition().toString());

        // System.out.println(limelight.hasPose());

         // If the pose exists and is within a 1m radius of current estimated position from odometry, update pose estimator with it
            /*if(!isMoving()&&limelight.hasPose()){
                TimestampPose2d currentPose = limelight.getTimestampedPose();
                
                if((getPose().getX()==0.0&&getPose().getY()==0.0)||(Math.sqrt(Math.pow(currentPose.getPose2d().getX() - m_poseEstimator.getEstimatedPosition().getX(), 2) + Math.pow(currentPose.getPose2d().getY() - m_poseEstimator.getEstimatedPosition().getY(), 2)) < 2)){
                    updatePose(currentPose);
                }                
            } */
    }


    //returns true if the swerve is moving
    public boolean isMoving(){
        return frontLeft.isMoving()&&frontRight.isMoving()&&backLeft.isMoving()&&backRight.isMoving();
    }

    //update the new pose of the limelight based on the limelight measurement

    //get the current position of the swerve drive as Pose2d
    public Pose2d getPose() {
        //System.out.println("AUTO: "+ m_poseEstimator.getEstimatedPosition());
        return m_poseEstimator.getEstimatedPosition();
    }

    //drive the motors
    //x is the forward, backward direction
    //y is the left and right
    //rot is the rotation of the robot
    public void drive(double xSpeed, double ySpeed, double rot) {
    
        double xSpeedCommanded;
        double ySpeedCommanded;

        // Gear ratio
        xSpeed *= ratio;
        ySpeed *= ratio;

        // Convert XY to polar for rate limiting
        double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
        double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

        // Calculate the direction slew rate based on an estimate of the lateral acceleration
        double directionSlewRate;
        if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
        } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
        }
        

        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - m_prevTime;
        double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
        if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
        else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
            // keep currentTranslationDir unchanged
            m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
            m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
            m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
        }
        else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        m_prevTime = currentTime;
        
        xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
        ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
        m_currentRotation = m_rotLimiter.calculate(rot);

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.maxSpeed;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.maxSpeed;
        double rotDelivered = m_currentRotation * DriveConstants.maxRotation;
    
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-gyro.getYaw())));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxSpeed);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void lockWheels(){
        frontLeft.setDesiredState(new SwerveModuleState(0,Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0,Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0,Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0,Rotation2d.fromDegrees(45)));
    }

    public void setWheelForward(){
        frontLeft.setForward();
        frontRight.setForward();
        backLeft.setForward();
        backRight.setForward();
    }

    //set the states of the max swerve
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.maxSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    //reset the odometry
    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(
            Rotation2d.fromDegrees(-gyro.getYaw()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
        pose);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.zeroYaw();
    }

    //switch the gear of the motor
    public void switchGear(double gear) {
        ratio = gear;
        System.out.println(gear);
    }

    //get the angle of the robot on the auto balancing thaang
    public double getAngle(){
        double pitch = gyro.getPitch();
        double roll = gyro.getRoll();
        return Math.cos(balanceDirection) * roll + Math.sin(balanceDirection) * pitch;
    }

    //check whether or not the robot has moved pass the auto balance thaang
    public boolean overTheThaang() {
        if(Math.abs(getAngle()) > 5) {
            over = true;
            avgAngle = 5.0;
        }
        if(balanceDirection == Double.POSITIVE_INFINITY) {
            balanceDirection = this.getPose().getRotation().getDegrees();
        }
        System.out.println(over);
        if(over) {
            System.out.println("avgAngle: " + avgAngle);
            avgAngle = (avgAngle *49)/50.0 + (Math.abs(getAngle()) * 1 / 50.0);
            System.out.println("avgAngle: " + avgAngle);
            if(avgAngle < 3.5) {
                return true;
            }
        }

        return false;
    }

    //auto balance autonomous code
    public void autoBalance(boolean fromPos) {
        double change = 1;
        if(fromPos){
            change = -1;
        }
        if(balanceDirection == Double.POSITIVE_INFINITY) {
            balanceDirection = this.getPose().getRotation().getDegrees();
        }

        double angle = getAngle();
        if(balancing && Math.abs(angle) < Constants.Autonomous.balanceOffAngle) {
            balancing = false;
        } else if(!balancing && Math.abs(angle) > Constants.Autonomous.balanceOnAngle) {
            balancing = true;
        }

        if(balancing) {
            double speed = Constants.Autonomous.balanceSpeed * Math.signum(angle) * change;
            drive(speed, 0, 0);
        } else {
            drive(0,0,0);
        }
    }

}