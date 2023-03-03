// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.commands.ResetPosition;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;


public class Drivetrain extends SubsystemBase {
  /** Creates a new TrainDrive. */
  public static double speedLimiter = 2.5; // the forward drive power gets divided by this value to reduce the speed
  public static double rotationLimiter = 2.75; // the rotational drive power gets divided by this value to reduce the speed

  // declar motors
  public final com.revrobotics.CANSparkMax leftMotor;
  public final com.revrobotics.CANSparkMax rightMotor;
  public final com.revrobotics.CANSparkMax leftMotor2;
  public final com.revrobotics.CANSparkMax rightMotor2;

  //public final com.revrobotics.CANSparkMax leftMotor2;
  
  public final DifferentialDriveOdometry m_odometry;
  public final DifferentialDriveKinematics m_kinematics;
  public final RamseteController m_ramseteController = new RamseteController(
  AutoPathConstants.kRamseteB_radSquaredPerMetersSquared, AutoPathConstants.kRamseteZeta_PerRad);
  
  //public final com.revrobotics.CANSparkMax rightMotor2;

  // declare encoders
  public final RelativeEncoder m_leftEncoder;
  public final RelativeEncoder m_rightEncoder;

  private final DifferentialDrive m_drive;

  public boolean m_reverseDrive = false;
  AHRS ahrs;
  
  public Drivetrain() {
    // define motors
    leftMotor = new com.revrobotics.CANSparkMax(Constants.DrivetrainConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    leftMotor2 = new com.revrobotics.CANSparkMax(Constants.DrivetrainConstants.LEFT_MOTOR2_ID, MotorType.kBrushless);
    rightMotor = new com.revrobotics.CANSparkMax(Constants.DrivetrainConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    rightMotor2 = new com.revrobotics.CANSparkMax(Constants.DrivetrainConstants.RIGHT_MOTOR2_ID, MotorType.kBrushless);

    // initialize motors
    motorInit(leftMotor, Constants.DrivetrainConstants.kLeftReversedDefault);
    motorInit(leftMotor2, Constants.DrivetrainConstants.kLeftReversedDefault);
    motorInit(rightMotor, Constants.DrivetrainConstants.kRightReversedDefault);
    motorInit(rightMotor2, Constants.DrivetrainConstants.kRightReversedDefault);

    leftMotor.setSmartCurrentLimit(Constants.DrivetrainConstants.STALL_LIMIT);
    rightMotor.setSmartCurrentLimit(Constants.DrivetrainConstants.STALL_LIMIT);
    leftMotor2.setSmartCurrentLimit(Constants.DrivetrainConstants.STALL_LIMIT);
    rightMotor2.setSmartCurrentLimit(Constants.DrivetrainConstants.STALL_LIMIT);

    leftMotor.setIdleMode(IdleMode.kBrake);
    leftMotor2.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kBrake);

    // group the left and right motors together as two groups
    leftMotor2.follow(leftMotor);
    rightMotor2.follow(rightMotor);
    
    // initialize encoders
    m_leftEncoder = leftMotor.getEncoder();
    m_rightEncoder = rightMotor.getEncoder();

    // initiailze drivetrain
    m_drive = new DifferentialDrive(leftMotor, rightMotor);

    // initialize NavX gyro
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex){
      DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
    }

    m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.kTrackWidth));
    m_odometry = new DifferentialDriveOdometry(
      ahrs.getRotation2d(), getLeftDistance(), getRightDistance());
    
  }

  public void motorInit(CANSparkMax motor, boolean invert) {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(Constants.DrivetrainConstants.kCurrentLimit);
    motor.setInverted(invert);

    encoderInit(motor.getEncoder());
  }

  private void encoderInit(RelativeEncoder encoder) {
    // set conversion factor and velocity factor (converting encoder ticks to real units)
    encoder.setPositionConversionFactor(Constants.DrivetrainConstants.kEncoderDistanceRatio);
    encoder.setVelocityConversionFactor(Constants.DrivetrainConstants.kHighSpeedPerPulseEncoderRatio);
    encoderReset(encoder);

  }

  public void resetAllEncoders(){
    encoderReset(m_rightEncoder);
    encoderReset(m_leftEncoder);
  }

  // resets encoder count/position
  public void encoderReset(RelativeEncoder encoder) {
    encoder.setPosition(0);
  }

  public double getLeftDistance() {
    return -m_leftEncoder.getPosition();
  }

  public double getRightDistance() {
    return -m_rightEncoder.getPosition();
  }

  private double getLeftSpeed() {
    return -m_leftEncoder.getVelocity();
  }

  private double getRightSpeed() {
    return -m_rightEncoder.getVelocity();
  }

  public double getAverageDistance() {
    return -(getRightDistance() + getLeftDistance())/2; 
  }

  public double getAverageSpeed() {
    return (getRightSpeed() + getLeftSpeed())/2;
  }

  public void resetGyroAngle(){
    ahrs.reset();
  }

  public double getPitch() {
    return ahrs.getYaw();
}
  public double getHeading() {
    return ahrs.getAngle();
  }
  public double getYaw() {
    return ahrs.getYaw() - 90;
  }

  // squares the MAGNITUDE of the value
  public static double sqaureInput(double input) {
    return Math.copySign(input * input, input);
  }

  public static boolean isTriggerPressed(double trigger) {
    return trigger > 0.95;
  }

  // positive stickY values moves forward
  // positive stickX values goes counterclockwise
  public void curvatureDrive(double stickY, double stickX, boolean stickButton) {
    m_drive.curvatureDrive(-stickY, -stickX, stickButton);
  }

  // Tankdrive command
  public void tankDrive(double leftPower, double rightPower, boolean squareInputs) {
    if (m_reverseDrive) {
      m_drive.tankDrive(-leftPower/2, -rightPower/2, squareInputs);
    }
    else {
      m_drive.tankDrive(-leftPower/2, -rightPower/2, squareInputs); 
    }
  }

  public void stopDrive() {
    m_drive.tankDrive(0, 0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition( ahrs.getRotation2d(),0,0,pose);
  }
  public DifferentialDriveWheelSpeeds getRamsetTargetWheelSpeeds(State tragectorySample) {
    return m_kinematics.toWheelSpeeds(m_ramseteController.calculate(getPose(), tragectorySample));
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
    m_drive.feed();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double newKP = SmartDashboard.getNumber("P value", 0.0);
    if (newKP != Constants.BalanceConstants.kP){
        Constants.BalanceConstants.kP = newKP;   
        SmartDashboard.putNumber("P value", Constants.BalanceConstants.kP);
    }
    double newKI = SmartDashboard.getNumber("I value", 0.0);
    if (newKI != Constants.BalanceConstants.kI){
        Constants.BalanceConstants.kI = newKI;  
        SmartDashboard.putNumber("I value", Constants.BalanceConstants.kI); 
    }
    double newKD = SmartDashboard.getNumber("D value", 0.0);
    if (newKD != Constants.BalanceConstants.kD){
        Constants.BalanceConstants.kD = newKD;   
        SmartDashboard.putNumber("D value", Constants.BalanceConstants.kD);
    }
  }
}
