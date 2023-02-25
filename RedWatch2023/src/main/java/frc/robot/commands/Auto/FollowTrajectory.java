package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.subsystems.Drivetrain;
import java.io.IOException;
import java.nio.file.Path;

public class FollowTrajectory extends CommandBase {

  private final Drivetrain drive;
  private Trajectory trajectory;
  private boolean toReset;

  public FollowTrajectory(Drivetrain drive, String trajectoryFilePath, boolean toReset) {
    this.drive = drive;
    this.toReset = toReset;
    addRequirements(drive);

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFilePath);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryFilePath,
          e.getStackTrace());
    }
  }

  @Override
  public void initialize() {
    if (toReset) {
      drive.resetOdometry(trajectory.getInitialPose());
    }

    final ProfiledPIDController thetaController =
        new ProfiledPIDController(
          AutoPathConstants.kPThetaController, 0, 0, AutoPathConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    new RamseteCommand(
        trajectory,
        drive::getPose,
        new RamseteController(Constants.AutoPathConstants.kRamseteB, Constants.AutoPathConstants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.AutoPathConstants.ksVolts,
                                   Constants.AutoPathConstants.kvVoltSecondsPerMeter,
                                   Constants.AutoPathConstants.kaVoltSecondsSquaredPerMeter),
        Constants.AutoPathConstants.kDriveKinematics,
        drive::getWheelSpeeds,
        new PIDController(Constants.AutoPathConstants.kPDriveVel, 0, 0),
        new PIDController(Constants.AutoPathConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        drive::tankDriveVolts,
        drive


    ); // Stops the robot

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(trajectory.getInitialPose());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}

