/**
 * >>> Author: Krithik Alluri
 * >>> Create Time: 2023-01-22 00:35:41
 * >>> Modified by: 2729StormRobotics
 * >>> Modified time: 2023-02-07 20:10:36
 * >>> Description: https://github.com/2729StormRobotics/RedWatch2023TestBot/tree/ShuffleBoardPathPlanner 
 
 */

package frc.robot.commands.Auto;

import java.nio.file.Path;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FollowPath extends CommandBase {

    private final Timer m_timer = new Timer();
    private final Trajectory m_trajectory;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;

    private final Drivetrain m_drivetrain;
    // Creates a new SimpleMotorFeedforward instance.

    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(AutoPathConstants.ksVolts,
            AutoPathConstants.kvVoltSecondsPerMeter, AutoPathConstants.kaVoltSecondsSquaredPerMeter);

    // Reset odometry to the starting pose of the trajectory.
    // Follow a path using the given trajectory.

    public FollowPath(Drivetrain Drivetrain, Trajectory trajectory) {
        m_trajectory = trajectory;
        m_drivetrain = Drivetrain;
        addRequirements(m_drivetrain);
    }

    public FollowPath(Drivetrain drivetrain, Path pathweaverJSON) {
        m_drivetrain = drivetrain;
        Trajectory trajectory;
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(pathweaverJSON);
        } catch (Exception e) {
            System.err.println(e.getMessage());
            trajectory = null;
        }
        m_trajectory = trajectory;
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        // m_drivetrain.m_field.getObject("traj").setTrajectory(m_trajectory);
        m_drivetrain.resetOdometry(m_trajectory.getInitialPose());
        m_prevTime = -1;
        var initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_drivetrain.m_kinematics.toWheelSpeeds(
                new ChassisSpeeds(
                        initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        m_timer.reset();
        m_timer.start();
        m_drivetrain.resetEncoders();
    }

    // Executes the driver.
    @Override
    public void execute() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;

        if (m_prevTime < 0) {
            m_drivetrain.tankDriveVolts(0.0, 0.0);
            m_prevTime = curTime;
            return;
        }

        State nextState = m_trajectory.sample(curTime);
        SmartDashboard.putNumber("Target Curature deg/m",
        Math.toDegrees(nextState.curvatureRadPerMeter));
        SmartDashboard.putNumber("Target Rotation deg",
        nextState.poseMeters.getRotation().getDegrees());

        // Ramset Controler

        DifferentialDriveWheelSpeeds wheelSpeeds = m_drivetrain.getRamsetTargetWheelSpeeds(nextState);

        //DifferentialDriveWheelSpeeds wheelSpeeds = m_drivetrain.m_kinematics
        //         .toWheelSpeeds(new ChassisSpeeds(nextState.velocityMetersPerSecond, 0, nextState.curvatureRadPerMeter));

        SmartDashboard.putNumber("Left Ramsette Setpoint", wheelSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("Right Ramsette Setpoint", wheelSpeeds.rightMetersPerSecond);

        // Feed Forward
        double leftFeedforward = m_feedforward.calculate(wheelSpeeds.leftMetersPerSecond,
                (wheelSpeeds.leftMetersPerSecond - m_prevSpeeds.leftMetersPerSecond) / dt);
        double rightFeedforward = m_feedforward.calculate(wheelSpeeds.rightMetersPerSecond,
                (wheelSpeeds.rightMetersPerSecond - m_prevSpeeds.rightMetersPerSecond) / dt);

        SmartDashboard.putNumber("Left FF Setpoint", leftFeedforward);
        SmartDashboard.putNumber("Right FF Setpoint", rightFeedforward);

        

        // SmartDashboard.putNumber("Left PID", leftPID);
        // SmartDashboard.putNumber("Right PID", rightPID);

        double leftOutput = leftFeedforward;
        double rightOutput = rightFeedforward ;

        SmartDashboard.putNumber("Left Output", leftOutput);
        SmartDashboard.putNumber("Right Output", rightOutput);

        m_drivetrain.tankDriveVolts(leftOutput, rightOutput);
        m_prevSpeeds = wheelSpeeds;
        m_prevTime = curTime;
    }

    // Stops the driver.
    @Override
    public void end(boolean interrupted) {
        m_timer.stop();

        if (interrupted) {
            m_drivetrain.tankDriveVolts(0.0, 0.0);
        }
    }
    // Checks if the timer has elapsed.

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}