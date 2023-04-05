// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.TelescopingConstants.HighExtendCone;
import static frc.robot.Constants.TelescopingConstants.HighExtendCube;
import static frc.robot.Constants.TelescopingConstants.LowExtendCone;
import static frc.robot.Constants.TelescopingConstants.LowExtendCube;
import static frc.robot.Constants.TelescopingConstants.MidExtendCone;
import static frc.robot.Constants.TelescopingConstants.MidExtendCube;
import static frc.robot.Constants.TelescopingConstants.SubstationCube;
import static frc.robot.Constants.TelescopingConstants.SubstationCone;
import static frc.robot.Constants.TelescopingConstants.fullIn;
import static frc.robot.Constants.TelescopingConstants.neutralPosTelescoping;
import static frc.robot.Constants.pinkArmConstants.kHighAngleCone;
import static frc.robot.Constants.pinkArmConstants.kHighAngleCube;
import static frc.robot.Constants.pinkArmConstants.kLowAngleCone;
import static frc.robot.Constants.pinkArmConstants.kLowAngleCube;
import static frc.robot.Constants.pinkArmConstants.kMidAngleCone;
import static frc.robot.Constants.pinkArmConstants.kMidAngleCube;
import static frc.robot.Constants.pinkArmConstants.kNeutralPos;
import static frc.robot.Constants.pinkArmConstants.kSubstationCone;
import static frc.robot.Constants.pinkArmConstants.kSubstationCube;

import java.util.function.ToDoubleBiFunction;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CommandGroups.IntakeCone;
import frc.robot.CommandGroups.IntakeCube;
import frc.robot.CommandGroups.IntakePos;
import frc.robot.CommandGroups.SetupConeHigh;
import frc.robot.CommandGroups.SetupScore;
import frc.robot.CommandGroups.TuckedInPos;
import frc.robot.CommandGroups.Auto.BackwardTime;
import frc.robot.CommandGroups.Auto.TwoPieceAuto;
import frc.robot.CommandGroups.Auto.TwoPieceHighAuto;
import frc.robot.commands.ArmOut;
import frc.robot.commands.ChangeGear;
import frc.robot.commands.Meltdown;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurnInPlacePID;
import frc.robot.commands.curvatureDrive;
import frc.robot.commands.Auto.HighConeTaxi;
import frc.robot.commands.Auto.HighCubeTaxi;
import frc.robot.commands.Auto.LowDock;
import frc.robot.commands.Auto.ScoreLowTaxi;
import frc.robot.commands.Auto.ScoreLowTaxiBalance;
import frc.robot.commands.AutoBalancing.AutoBalancePID;
import frc.robot.commands.AutoBalancing.AutoBalancePIDRewrite;
import frc.robot.commands.Gripper.EjectItem;
import frc.robot.commands.Gripper.RunIntake;
import frc.robot.commands.Gripper.StopGripper;
import frc.robot.commands.Gripper.StopPulse;
import frc.robot.commands.Gripper.PulseIntake.PulseIntake;
import frc.robot.commands.Lights.animateCandle;
import frc.robot.commands.TelescopingArmCommands.ArmControl;
import frc.robot.commands.Vision.AprilTagMode;
import frc.robot.commands.Vision.ReflectiveTapeMode;
import frc.robot.commands.Vision.VisionAlign;
import frc.robot.commands.Vision.VisionAlignHighCone;
import frc.robot.commands.Vision.VisionAlignMidCone;
import frc.robot.commands.pivotArm.armJoint;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controller
  private final XboxController m_driver = new XboxController(Constants.DrivetrainConstants.kDriverControllerPort);
  private final XboxController m_weapons = new XboxController(Constants.DrivetrainConstants.kWeaponsControllerPort);

  private SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(2); // controls acceleration of forward speed
  private SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(1.1); // controls acceleration of rotational speed
  private SlewRateLimiter m_pivotLimiter = new SlewRateLimiter(2.4);

  // Subsystems
  private final Lights m_lights;
  private final Gripper m_gripper;
  private final Drivetrain m_drivetrain;
  private final PivotArm m_PinkArm;
  private final TelescopingArm m_arm;
  private final Vision m_Vision;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Subsystems Instantiation
    m_gripper = new Gripper();
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

    m_Vision.setDefaultCommand(new AprilTagMode(m_Vision));

    // Control Panel
    new ControlPanel(m_drivetrain, m_gripper, m_lights, m_PinkArm, m_arm);

    // Lights
    
    // Gripper
    // m_gripper.setDefaultCommand(new PulseIntake(m_gripper));

    // sets the drivetrain default command to curvatureDrive, with the
    // slewratelimiters
    // Left Joystick: forwards/backward, Right Joystick: turn in place left/right
    m_drivetrain.setDefaultCommand(
        new curvatureDrive(
            () -> Math.copySign(Constants.DrivetrainConstants.kS, m_driver.getLeftY())
                + m_forwardLimiter.calculate(m_driver.getLeftY() / Drivetrain.speedLimiter),
            () -> Math.copySign(Constants.DrivetrainConstants.kS, m_driver.getRightX())
                + m_rotationLimiter.calculate(m_driver.getRightX() / Drivetrain.rotationLimiter),
            () -> true, m_drivetrain));

    // Pink Arm
    m_PinkArm.setDefaultCommand(
        new armJoint(() -> m_pivotLimiter.calculate(m_weapons.getRightY()), m_PinkArm));
    m_lights.setDefaultCommand(
        new animateCandle(m_lights, m_weapons));
    // m_lights.setGiven(255, 255, 255);

    // Vision
    m_Vision.setDefaultCommand(new AprilTagMode(m_Vision));
    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be\
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Testing Button (try cmds on me! NO OTHER BUTTONS PLS!)


    new JoystickButton(m_driver, Button.kStart.value).onTrue(new
    Meltdown(m_drivetrain, m_gripper, m_PinkArm, m_arm));

    // !!!!!
    // PLEASE DO NOT CHANGE THESEWIHTOUT ASKING, AKSHAY WILL BE MAD!!!!!
    // !!!!!

    /*
     * DRIVER
     */
    // Right Bumper: Intake Cube
    new JoystickButton(m_driver, Button.kRightBumper.value).onTrue(new RunIntake(m_gripper, false));
    // Left Bumper: Intake Cone
    new JoystickButton(m_driver, Button.kLeftBumper.value).onTrue(new RunIntake(m_gripper, true));
    // Right Trigger: Eject Cube
    new Trigger(() -> (m_driver.getRightTriggerAxis() > 0.5))
        .onTrue(new EjectItem(m_gripper, Constants.GripperConstants.kGripperEjectCubeSpeed));
    // Left Trigger: Eject Cone
    new Trigger(() -> (m_driver.getLeftTriggerAxis() > 0.5))
        .onTrue(new EjectItem(m_gripper, Constants.GripperConstants.kGripperEjectConeSpeed));
    // Y: Stop Gripper
    new JoystickButton(m_driver, Button.kY.value).onTrue(new StopGripper(m_gripper));
    // B: Change Gear
    new JoystickButton(m_driver, Button.kB.value).onTrue(new ChangeGear());
    // X: Vision Align High
    new JoystickButton(m_driver, Button.kX.value).whileTrue(new VisionAlignHighCone(m_drivetrain, m_Vision));
    // A: Vision Align Mid
    new JoystickButton(m_driver, Button.kA.value).whileTrue(new VisionAlignMidCone(m_drivetrain, m_Vision));
    // BACK: Stop Gripper Pulsing
    new JoystickButton(m_driver, Button.kBack.value).onTrue(new StopPulse(m_gripper));

    /**
     * WEAPONS
     */
    // A: Mid Cone Setup
    new JoystickButton(m_weapons, Button.kA.value)
        .onTrue(new SetupScore(m_PinkArm, m_arm, kMidAngleCone, MidExtendCone));
    // B: High Cone Setup
    new JoystickButton(m_weapons, Button.kB.value).onTrue(new SetupConeHigh(m_arm, m_PinkArm)); // testme
    // Y: High Cube Setup
    new JoystickButton(m_weapons, Button.kY.value)
        .onTrue(new SetupScore(m_PinkArm, m_arm, kHighAngleCube, HighExtendCube));
    // X: Mid Cube Setup
    new JoystickButton(m_weapons, Button.kX.value)
        .onTrue(new SetupScore(m_PinkArm, m_arm, kMidAngleCube, MidExtendCube));
    // Start: Substation Cube Intake Setup
    new JoystickButton(m_weapons, Button.kStart.value)
        .onTrue(new SetupScore(m_PinkArm, m_arm, kSubstationCube, SubstationCube));
    // RB: Intake Cone Setup
    new JoystickButton(m_weapons, Button.kRightBumper.value)
        .onTrue(new IntakePos(m_PinkArm, m_arm, kLowAngleCone, LowExtendCone));
    // LB: Intake Cube Setup
    new JoystickButton(m_weapons, Button.kLeftBumper.value)
        .onTrue(new IntakePos(m_PinkArm, m_arm, kLowAngleCube, LowExtendCube));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return null;

    //one low one high
    // return new TwoPieceAuto(m_drivetrain, m_gripper, m_arm, m_PinkArm);
    
    // return new LowDock(m_drivetrain, m_PinkArm);

    // return new HighCubeTaxi(m_drivetrain, m_gripper, m_PinkArm, m_arm);
    return new HighConeTaxi(m_drivetrain, m_gripper, m_PinkArm, m_arm);

    // return new TwoPieceHighAuto(m_drivetrain, m_gripper, m_arm, m_PinkArm, m_Vision);
  
}
}