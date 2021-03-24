
package frc.team1918.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;

public class AutonSubsystem extends SubsystemBase {
  public AutonSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public TrajectoryConfig getTrajectoryConfig() {
    TrajectoryConfig config = new TrajectoryConfig(Constants.Auton.kMaxSpeedMetersPerSecond, Constants.Auton.kMaxAccelerationMetersPerSecondSquared);
      config.setKinematics(Constants.Swerve.kDriveKinematics); //Add the kinematics to ensure max speed is obeyed
      // config.addConstraint(autoVoltageConstraint); //Apply a voltage constraint -- see example from wpilib
    return config;
  }

  public Trajectory getTrajectoryByName(String pathname) {
    var config = getTrajectoryConfig();
    var startPos = new Pose2d(0, 0, new Rotation2d(0));
    var endPos = new Pose2d(0, 0, new Rotation2d(0));
    var intWaypoints = new ArrayList<Translation2d>();
    switch (pathname) {
      case "test":
        endPos = new Pose2d(3, 0, new Rotation2d(0)); //set a new endPos
        intWaypoints.add(new Translation2d(1, 1));
        intWaypoints.add(new Translation2d(2,-1));
        break;
      default:
        Helpers.Debug.debug("Invalid auton path name requested: "+pathname);
        //Going nowhere in life, doing nothing.
        break;
    }
    return TrajectoryGenerator.generateTrajectory(startPos, intWaypoints, endPos, config);
  }
}
