
package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class Autos extends CommandBase {
  TrajectoryConfig config;
  Trajectory trajectory;
  ProfiledPIDController thetaController;
  SwerveControllerCommand swerveControllerCommand;
  SwerveDrive m_robotDrive;
  List<PathPlannerTrajectory> pathGroup;

  //Limelight giving correct value
  public Command toPosition(Pose2d position, boolean limelight) {
    DriverStation.Alliance color;
    color = DriverStation.getAlliance();
    PathPlannerTrajectory pathToPosition;
    if(color == DriverStation.Alliance.Blue){
      pathToPosition = PathPlanner.generatePath(
        new PathConstraints(2.5, 2.5),
        new PathPoint(new Translation2d(m_robotDrive.getPose().getX(), m_robotDrive.getPose().getY()), Rotation2d.fromDegrees(180)),
        new PathPoint(new Translation2d(position.getX(), position.getY()), Rotation2d.fromDegrees(180))
      );
      System.out.println("starting:"+m_robotDrive.getPose().toString());
      System.out.println("ending:"+position.toString());

    } else {
      double startY = m_robotDrive.getPose().getY();
      if(limelight){
        startY = Constants.Autonomous.FIELD_HEIGHT - startY;
      }
      m_robotDrive.resetOdometry(new Pose2d(m_robotDrive.getPose().getX(), startY, m_robotDrive.getPose().getRotation()));
      double difference = position.getY() - m_robotDrive.getPose().getY();

      pathToPosition = PathPlanner.generatePath(
        new PathConstraints(2.5, 2.5),
        new PathPoint(new Translation2d(m_robotDrive.getPose().getX(), startY), Rotation2d.fromDegrees(180)),
        new PathPoint(new Translation2d(position.getX(), m_robotDrive.getPose().getY() - difference), Rotation2d.fromDegrees(180))
      );
      System.out.println("starting:"+m_robotDrive.getPose().toString());
      System.out.println("ending:"+position.toString());

    }

    PIDController PIDX = new PIDController(2, 0.0, 0.0);
    PIDController PIDY = new PIDController(3, 0.0, 0.0);
    PIDController PIDRotation = new PIDController(0, 0.0, 0.0);
    PIDRotation.enableContinuousInput(-Math.PI, Math.PI);

    return new PPSwerveControllerCommand(
        pathToPosition,
        m_robotDrive::getPose,
        Constants.DriveConstants.kDriveKinematics,
        PIDX, // PID constants to correct for translation error (used to create the X and Y
        PIDY,  // PID controllers)
        PIDRotation,
        m_robotDrive::setModuleStates,
        true,
        m_robotDrive
      );
  }

  public HashMap<String,Command> getEventMap(){
    HashMap<String, Command> eventMap = new HashMap<>();
    
    /*eventMap.put("releaseObj", new SequentialCommandGroup(
        new InstantCommand(() -> claw.releaseObjectAuto()),
        new WaitCommand(1),
        new InstantCommand(() -> claw.stop())));*/
        
    eventMap.put("wait", new WaitCommand(0.2));
  
    /*eventMap.put("bringArmHome", new InstantCommand(() -> {
      arm.setElbowBase();
    }).andThen(new WaitCommand(0.7)).andThen(new InstantCommand(() -> arm.setShoulderBase())));*/
    /*eventMap.put("extendArm", new InstantCommand(() -> arm.highPlatformCube()));*/
    return eventMap;
    
  }
}
