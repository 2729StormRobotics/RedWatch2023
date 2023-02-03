// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button; 
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.curvatureDrive;
import frc.robot.commands.differentialDrive;
import frc.robot.commands.Gripper.CheckObjectForColorChange;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.Constants.LightConstants.*;
import frc.robot.commands.Gripper.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // Controller
  private final XboxController m_driver = new XboxController(Constants.kDriverControllerPort);
  private final XboxController m_weapons = new XboxController(Constants.kWeaponsControllerPort);

  private SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(1); // controls acceleration of forward speed
  private SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(0.5); // controls acceleration of rotational speed

  // Subsystems
  private final Lights m_lights;
  private final Gripper m_gripper;
  private final Drivetrain m_drivetrain;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems Instantiation
    m_gripper = new Gripper();
    m_lights = new Lights();
    m_drivetrain = new Drivetrain();

    // Setting default commands

    // Lights
    m_lights.setDefaultCommand(new CheckObjectForColorChange(m_lights, m_gripper));

    // sets the drivetrain default command to curvatureDrive, with the slewratelimiters
    // Left Joystick: forwards/backward, Right Joystick: turn in place left/right
    m_drivetrain.setDefaultCommand(
    new curvatureDrive(
      () -> Math.copySign(Constants.kS, m_driver.getLeftY())
      + m_forwardLimiter.calculate(m_driver.getLeftY() / Drivetrain.speedLimiter), 
      () -> Math.copySign(Constants.kS, m_driver.getRightX()) 
      + m_rotationLimiter.calculate(m_driver.getRightX() / Drivetrain.rotationLimiter),
      () -> true, m_drivetrain));
    // Configure the button bindings

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  // Sets the button bindings on the controller
  // B brakes the drivetrain
  private void configureButtonBindings() {
    new JoystickButton(m_driver, Button.kB.value).whileTrue(
      new differentialDrive(() -> 1, () -> 1, () -> 0.0, () -> 0.0, m_drivetrain));
    
    new JoystickButton(m_weapons, Button.kBack.value).whenHeld(new IntakeItem(m_gripper));
    new JoystickButton(m_weapons, Button.kStart.value).whenHeld(new EjectItem(m_gripper));
    
    new JoystickButton(m_weapons, Button.kLeftStick.value).whenPressed(new ChangeColor(m_lights, kYellowCone));
    new JoystickButton(m_weapons, Button.kRightStick.value).whenPressed(new ChangeColor(m_lights, kPurpleCube));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
