// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandGroups.AutoBalance_x6;
import frc.robot.CommandGroups.AutoScoreSetup;
import frc.robot.CommandGroups.IntakePos;

import frc.robot.CommandGroups.BalanceFromDistance;
import frc.robot.CommandGroups.Dunk;
import frc.robot.CommandGroups.IntakeCone;
import frc.robot.CommandGroups.IntakeCube;
import frc.robot.CommandGroups.ParallelAutoScoreSetup;
import frc.robot.CommandGroups.Auto.B1_A;
import frc.robot.CommandGroups.Auto.B1_Testing;
import frc.robot.Constants.TelescopingConstants;
import frc.robot.commands.pivotArm.PivotPID;
import frc.robot.commands.pivotArm.armJoint;
import frc.robot.commands.pivotArm.turnToDegrees;
import frc.robot.subsystems.TelescopingArm;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.PivotArm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.AutoForwardPID;
import frc.robot.commands.ChangeGear;
import frc.robot.commands.ResetPosition;
import frc.robot.commands.TurnInPlace;
import frc.robot.commands.TurnInPlacePID;
import frc.robot.commands.curvatureDrive;
import frc.robot.commands.Auto.FollowPath;
import frc.robot.commands.AutoBalancing.AutoBalance;
import frc.robot.commands.AutoBalancing.AutoBalancePID;
import frc.robot.commands.AutoBalancing.ForwardUntilTilted;
import frc.robot.commands.Lights.ChangeColor;
import frc.robot.commands.Lights.CheckObjectForColorChange;
import frc.robot.commands.Lights.animateCandle;
import frc.robot.commands.TelescopingArmCommands.ArmControl;
import frc.robot.commands.TelescopingArmCommands.ExtendVal;
import frc.robot.commands.TelescopingArmCommands.MaintainPos;
import frc.robot.commands.Vision.AprilTagMode;
import frc.robot.commands.Vision.FollowTarget;
import frc.robot.commands.Vision.VisionAlign;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.Constants.LightConstants.*;
import frc.robot.commands.Gripper.*;
import static frc.robot.Constants.pinkArmConstants.*;

import java.util.List;

import static frc.robot.Constants.TelescopingConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controller
  private final XboxController m_driver = new XboxController(Constants.DrivetrainConstants.kDriverControllerPort);
  private final XboxController m_weapons = new XboxController(Constants.DrivetrainConstants.kWeaponsControllerPort);

  private SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(2); // controls acceleration of forward speed
  private SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(1.5); // controls acceleration of rotational speed

  // Subsystems
  private final Lights m_lights;
  // private final Gripper m_gripper;
  private final Drivetrain m_drivetrain;
  private final PivotArm m_PinkArm;
  private final TelescopingArm m_arm;
  private final Vision m_Vision;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems Instantiation
    // m_gripper = new Gripper();
    m_lights = new Lights();
    m_drivetrain = new Drivetrain();
    m_PinkArm = new PivotArm();
    m_arm = new TelescopingArm();
    m_Vision = new Vision();

    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putNumber("Speed Limiter", Drivetrain.speedLimiter);

    // Setting default commands
    m_arm.setDefaultCommand(
      new ArmControl(() -> m_weapons.getLeftY(), m_arm));

    // Control Panel
    // new ControlPanel(m_drivetrain, m_gripper, m_lights, m_PinkArm, m_arm);

    // Lights
    // m_lights.setDefaultCommand(new CheckObjectForColorChange(m_lights, m_gripper));

    // sets the drivetrain default command to curvatureDrive, with the slewratelimiters
    // Left Joystick: forwards/backward, Right Joystick: turn in place left/right
    m_drivetrain.setDefaultCommand(
    new curvatureDrive(
      () -> Math.min(
        Math.copySign(Constants.DrivetrainConstants.kS, m_driver.getLeftY())
      + m_forwardLimiter.calculate(m_driver.getLeftY() / Drivetrain.speedLimiter), 0.5),
      () -> Math.copySign(Constants.DrivetrainConstants.kS, m_driver.getRightX()) 
      + m_rotationLimiter.calculate(m_driver.getRightX() / Drivetrain.rotationLimiter),
      () -> true, m_drivetrain));
    
    // Pink Arm
    // m_PinkArm.setDefaultCommand(
    //   new armJoint(() -> m_weapons.getRightY(), m_PinkArm)
    // );
    m_lights.setDefaultCommand(
      new animateCandle(m_lights, m_driver)
    );
    // Configure the button bindings

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {    
    // new JoystickButton(m_driver, Button.kLeftBumper.value).onTrue(new IntakeCube(m_gripper));
    // new JoystickButton(m_driver, Button.kRightBumper.value).onTrue(new IntakeCone(m_gripper));
    // new JoystickButton(m_driver, Button.kX.value).onTrue(new EjectItem(m_gripper));
    // new JoystickButton(m_driver, Button.kY.value).onTrue(new StopGripper(m_gripper));
    // new JoystickButton(m_driver, Button.kA.value).onTrue(new BalanceFromDistance(m_drivetrain, false));
    new JoystickButton(m_driver, Button.kStart.value).onTrue(new ResetPosition(m_drivetrain));
    new JoystickButton(m_driver, Button.kB.value).onTrue(new ChangeGear());
    new JoystickButton(m_driver, Button.kY.value).onTrue(new BalanceFromDistance(m_drivetrain, false));
    new JoystickButton(m_driver, Button.kX.value).onTrue(new B1_A(m_drivetrain, m_Vision));
    new JoystickButton(m_driver, Button.kA.value).whileTrue(new VisionAlign(m_drivetrain, m_Vision));
    new JoystickButton(m_driver, Button.kBack.value).onTrue(new ResetPosition ( m_drivetrain));
  //   new JoystickButton(m_weapons, Button.kLeftStick.value).onTrue(new animateCandle(m_lights, m_weapons));   
  //   // new JoystickButton(m_weapons, Button.kY.value).toggleOnTrue(new ExtendVal( TelescopingConstants.HighExtendCube, m_arm));
  //   //new JoystickButton(m_weapons, Button.kX.value).toggleOnTrue(new ExtendVal( TelescopingConstants.MidExtendCube, m_arm));
  //   //new JoystickButton(m_weapons, Button.kA.value).toggleOnTrue(new ExtendVal( TelescopingConstants.LowExtendHybrid , m_arm));
    
  //   //cube high 
  //    new JoystickButton(m_weapons, Button.kY.value).onTrue(new ParallelAutoScoreSetup(m_PinkArm, m_arm, m_gripper, kHighAngleCube, HighExtendCube));
  //   //cube mid
  //    new JoystickButton(m_weapons, Button.kX.value).onTrue(new ParallelAutoScoreSetup(m_PinkArm, m_arm, m_gripper, kMidAngleCube, MidExtendCube));
    
  //   //Cone high 
  //  // new JoystickButton(m_weapons, Button.kB.value).onTrue(new ParallelAutoScoreSetup(m_PinkArm, m_arm, m_gripper, kHighAngleCone, HighExtendCone));
  //   //cone mid
  //   //new JoystickButton(m_weapons, Button.kA.value).onTrue(new ParallelAutoScoreSetup(m_PinkArm, m_arm, m_gripper, kMidAngleCone, MidExtendCone));
    
  //   //intake cone
  //   new JoystickButton(m_weapons, Button.kRightBumper.value).onTrue(new ParallelAutoScoreSetup(m_PinkArm, m_arm, m_gripper, kLowAngleCone, LowExtendCone));
  //   // intake cube
  //   new JoystickButton(m_weapons, Button.kLeftBumper.value).onTrue(new ParallelAutoScoreSetup(m_PinkArm, m_arm, m_gripper, kLowAngleCube, LowExtendCube));
    

    //dunk
    // new JoystickButton(m_weapons, Button.kB.value).onTrue(new Dunk(m_PinkArm, m_arm, m_gripper));

    //TESTING THE NEUTRAL POSITION
    // new JoystickButton(m_weapons, Button.kLeftBumper.value).onTrue(new IntakePos(m_PinkArm, m_arm, m_gripper, fullIn, kNeutralPos, neutralPosTelescoping));

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Trajectory traj) {
    // Create a voltage constraint to ensure we don't accelerate too fast

    // var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             Constants.AutoPathConstants.ksVolts,
    //             Constants.AutoPathConstants.kvVoltSecondsPerMeter,
    //             Constants.AutoPathConstants.kaVoltSecondsSquaredPerMeter),
    //         Constants.AutoPathConstants.kDriveKinematics,
    //         3  );

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoPathConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoPathConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.AutoPathConstants.kDriveKinematics);
            // Apply the voltage constraint
            // .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);
    m_drivetrain.m_field.getObject("traj").setTrajectory(traj);
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            traj,
            m_drivetrain::getPose,
            new RamseteController(Constants.AutoPathConstants.kRamseteB, Constants.AutoPathConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.AutoPathConstants.ksVolts,
                Constants.AutoPathConstants.kvVoltSecondsPerMeter,
                Constants.AutoPathConstants.kaVoltSecondsSquaredPerMeter),
            Constants.AutoPathConstants.kDriveKinematics,
            m_drivetrain::getWheelSpeeds,
            new PIDController(Constants.AutoPathConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.AutoPathConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_drivetrain::tankDriveVolts,
            m_drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(traj.getInitialPose());

    // Run path following command, then stop at the end.
    // return new B1_Testing(m_drivetrain);
    return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
  }
}

