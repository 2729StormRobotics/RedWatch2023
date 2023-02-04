// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button; 
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TelescopingConstants;
import frc.robot.commands.ExtendVal;
import frc.robot.subsystems.MeasuringPotentiometer;
import frc.robot.subsystems.TelescopingArm;

import static frc.robot.Constants.IOPorts.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Controller
  private final XboxController m_driver = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final XboxController m_weapons = new XboxController(Constants.OperatorConstants.kWeaponsControllerPort);

  // Subsystems
  private final TelescopingArm m_arm;
  private final MeasuringPotentiometer m_pot;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems Instantiation
    m_arm = new TelescopingArm();
    m_pot = new MeasuringPotentiometer();

    // Setting default commands

  
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    new JoystickButton(m_weapons, Button.kY.value).toggleOnTrue(new ExtendVal(false, TelescopingConstants.HighExtendCube,m_pot, m_arm));
    new JoystickButton(m_weapons, Button.kX.value).toggleOnTrue(new ExtendVal(false, TelescopingConstants.MidExtendCube,m_pot, m_arm));
    new JoystickButton(m_weapons, Button.kA.value).toggleOnTrue(new ExtendVal(true, TelescopingConstants.LowStop ,m_pot, m_arm));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
