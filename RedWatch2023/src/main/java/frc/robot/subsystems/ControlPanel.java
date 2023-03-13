// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Lights.*;
import frc.robot.commands.pivotArm.SetEncoder;

import static frc.robot.Constants.ControlPanelConstants;
import static frc.robot.Constants.LightConstants;

import java.util.Map;

public class ControlPanel extends SubsystemBase {

  private final ShuffleboardTab m_controlpanelTab;

  private final ShuffleboardLayout m_drivetrainStatus;
  private final ShuffleboardLayout m_gripperStatus;
  private final ShuffleboardLayout m_lightsStatus;
  private final ShuffleboardLayout m_pivotArmStatus;
  private final ShuffleboardLayout m_telescopingArmStatus;

  private final GenericEntry setLightColor_R;
  private final GenericEntry setLightColor_G;
  private final GenericEntry setLightColor_B;

  private final Drivetrain m_drivetrain;
  private final Gripper m_gripper;
  private final Lights m_lights;
  private final PivotArm m_pivotArm;
  private final TelescopingArm m_telescopingArm;

  /** Creates a new ControlPanel. */
  public ControlPanel(Drivetrain drivetrain, Gripper gripper, Lights lights, PivotArm pivotArm, TelescopingArm telescopingArm) {
    m_drivetrain = drivetrain;
    m_gripper = gripper;
    m_lights = lights;
    m_pivotArm = pivotArm;
    m_telescopingArm = telescopingArm;

    m_controlpanelTab = Shuffleboard.getTab(ControlPanelConstants.kShuffleboardTab);

    m_drivetrainStatus = m_controlpanelTab.getLayout("Drivetrain Status", BuiltInLayouts.kList)
      .withProperties(Map.of("Label position", "TOP"))
      .withPosition(0, 0)
      .withSize(2, 4);
      
    m_gripperStatus = m_controlpanelTab.getLayout("Gripper Status", BuiltInLayouts.kList)
      .withProperties(Map.of("Label position", "TOP"))
      .withPosition(2, 0)
      .withSize(2, 2);

    m_lightsStatus = m_controlpanelTab.getLayout("Light Status", BuiltInLayouts.kList)
      .withProperties(Map.of("Label position", "TOP"))
      .withPosition(4, 0)
      .withSize(2, 3);

    m_pivotArmStatus = m_controlpanelTab.getLayout("Pivot Arm Status", BuiltInLayouts.kList)
      .withProperties(Map.of("Label position", "TOP"))
      .withPosition(8, 0)
      .withSize(1, 2);
    
    m_telescopingArmStatus = m_controlpanelTab.getLayout("Telescoping Arm Status", BuiltInLayouts.kList)
      .withProperties(Map.of("Label position", "TOP"))
      .withPosition(10, 0)
      .withSize(1, 2);

    m_drivetrainStatus.addNumber("Average Speed", () -> m_drivetrain.getAverageSpeed()); // How fast the robot is
    m_drivetrainStatus.addNumber("Left Position", () -> m_drivetrain.getLeftDistance()); // How far the robot is
    m_drivetrainStatus.addNumber("Right Position", () -> m_drivetrain.getRightDistance());
    m_drivetrainStatus.addNumber("Pitch", () -> m_drivetrain.getPitch()); // Pitch of robot
    m_drivetrainStatus.addNumber("Yaw", () -> m_drivetrain.getYaw());

    m_lightsStatus.addNumber("R", () -> m_lights.R);
    m_lightsStatus.addNumber("G", () -> m_lights.G);
    m_lightsStatus.addNumber("B", () -> m_lights.B);
    setLightColor_R = m_lightsStatus.add("Light Input R", LightConstants.kDefaultColor).getEntry();
    setLightColor_G = m_lightsStatus.add("Light Input G", LightConstants.kDefaultColor).getEntry();
    setLightColor_B = m_lightsStatus.add("Light Input B", LightConstants.kDefaultColor).getEntry();
    // m_lightsStatus.add(new ChangeColor(m_lights, setLightColor.get().getDouble()));

    m_gripperStatus.addString("Gripper Mode", () -> Gripper.m_gripper_direction);
    // m_gripperStatus.addNumber("R", () -> m_gripper.m_detectedColor.red);
    // m_gripperStatus.addNumber("Green", () -> m_gripper.m_detectedColor.green);
    // m_gripperStatus.addNumber("Blue", () -> m_gripper.m_detectedColor.blue);
    // m_gripperStatus.addNumber("Proximity", () -> m_gripper.m_proximity);
    // m_gripperStatus.addBoolean("Purple", () -> m_gripper.isPurple());
    // m_gripperStatus.addBoolean("Yellow", () -> m_gripper.isYellow());
    m_gripperStatus.addNumber("Gripper Velocity", () -> m_gripper.getVelocity());

    m_pivotArmStatus.addNumber("Pivot Encoder", () -> m_pivotArm.getAngle());
    

    m_telescopingArmStatus.addNumber("Telescoping Encoder", () -> m_telescopingArm.getArmDistance());
    m_telescopingArmStatus.addNumber("Pot value", () -> m_telescopingArm.pot_val);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
