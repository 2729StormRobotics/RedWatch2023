package frc.robot.subsystems;

import static frc.robot.Constants.TelescopingConstants.*;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TelescopingArmCommands.ResetPot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class TelescopingArm extends SubsystemBase {
    
public final CANSparkMax m_ArmExtend;// Calculates the potential of the control panel.
public final AnalogPotentiometer pot;
private final ShuffleboardTab m_controlPanelTab;
private final ShuffleboardLayout m_controlPanelStatus; 
public double pot_val;
public double offset = 0;
public final RelativeEncoder m_ArmEncoder;

/**
 * Controls Telescoping Mechanism
 */

 public TelescopingArm() {
    
    //Initializes the arm encoder.
    m_ArmExtend = new CANSparkMax(kArmExtendPort, MotorType.kBrushless);
    // setMotor(motor, INVERSE);
    setMotor(m_ArmExtend, true);
    m_ArmEncoder = m_ArmExtend.getEncoder();
    positionEncoderInit(m_ArmEncoder);
    // Initialize the shuffleboard.
    pot = new AnalogPotentiometer(1);
    m_controlPanelTab = Shuffleboard.getTab("stringpot");
    m_controlPanelStatus = m_controlPanelTab.getLayout("String Pot", BuiltInLayouts.kList)
    .withSize(3, 3)
    .withProperties(Map.of("Label position", "TOP"));

    shuffleboardInit();
  /*
    try {
      m_ahrs = new AHRS(SPI.Port.kMXP);
    } 
    catch (RuntimeException ex){
      DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
    }*/
  }
// Gets the distance from this value to this value.
  public double getDistance(){
    return pot_val;
  }

  // Initialize the shuffleboard.
  private void shuffleboardInit() {
    // Proximity to ball
    m_controlPanelStatus.addNumber("Arm Length", () -> pot_val);
    m_controlPanelStatus.addNumber("Pot Offset", () -> offset);
    m_controlPanelStatus.add(new ResetPot(this));
  }
  public void changeMode(String mode) {

  }

  // Turns the motor on or off.
  public void turnMotor(CANSparkMax motor, boolean inverse) {
    if (inverse) {
      motor.set(-ArmSpeed);
    }
    else {
      motor.set(ArmSpeed);
    }
  }

  // Initializes the position encoder.
  private void positionEncoderInit(RelativeEncoder encoder) {
    encoder.setPositionConversionFactor(kDistancePerRevolution);

    encoderReset(encoder);
  }

  // private void pivotEncoderInit(RelativeEncoder encoder) {
  //   encoder.setPositionConversionFactor(kAnglePerRevolution);
  // }

  // Resets the encoder to its initial position.
  public void encoderReset(RelativeEncoder encoder) {
    encoder.setPosition(0);
  }


  // Returns the distance between the encoding and the arm.
  public double getArmDistance() {
    return -m_ArmEncoder.getPosition();
  }

  // public double getRightPivot() {
  //   return -m_pivotArmEncoder.getPosition();
  // }

  // Sets the motor.
  public void setMotor(CANSparkMax motor, boolean inverse) {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(inverse);
  }
/* 
  public double getGyroAngle(){
    return m_ahrs.getAngle();
  }

  public void resetGyroAngle(){
    m_ahrs.reset();
  }
*/

  // Periodically calculates the value of the pot.
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // pot_val = ((pot.get())*50)-offset;
    pot_val = ((pot.get())*50);
  }
 }

