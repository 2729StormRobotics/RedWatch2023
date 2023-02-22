// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Robo Rio/PDP ports can be referenced at 
 * https://docs.google.com/spreadsheets/d/1bbRh-F-XOhQwSzRBP7F1Jo3ygR4dRqCi2I8FiAghLpA/edit#gid=0
 */

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
 
  public static final class TelescopingConstants {
    public static final double MidExtendCube = 14; // previous val 23.5 actual val 13.909128
    public static final double HighExtendCube = 27; //Actual distance 26.938031
    public static final double HighExtendCone = 27.5; //actual distance 27.697191
    public static final double MidExtendCone = 21; //Actual distance 20.860632
    //Low value for turn might change when testing
    public static final double LowExtendHybrid = 14;
    //Add values for the low hybrid node

    public static final double potLowStop = 0.037;
    public static final double potHighStop = 0.62;
    public static final double Tolerance = 0.5;
    public static final double ArmSpeed = 0.25;
    public static final int kArmExtendPort = 4;
    // pivoting gearbox = 1:125 
    public static final double kTelescopingGearRatio = 1.0 / 12.0;

    public static final double kDistancePerRevolution = kTelescopingGearRatio * (7.0 / 8.0) * 3.14; //TODO: put in gear ratio for the climbers!!!
}
  public static final class pinkArmConstants {
    // Can change the port of the motors
    public static final int kRightPivotPort = 8;
    public static final int kLeftPivotPort = 3;
    // pivoting gearbox = 1:125 
    public static final double kTelescopingGearRatio = 1.0 / 16.0;
    public static final double kPivotingGearRatio = 1.0 / 421.875;
    public static final double kDistancePerRevolution = kTelescopingGearRatio * (7.0 / 8.0) * 3.14; //TODO: put in gear ratio for the climbers!!!
    public static final double kEncoderOffset = 300; // figure this out
    public static final double kClimberRightSize = 12.0;
    // public static final double kAnglePerRevolution = kPivotingGearRatio * 3.14;
    public static final int kCurrentLimit = 60;
    public static final int kStallLimit = 45;
    //Sets the speed of the pivot arm, needs to be changed depending on the gear ratio for the pivot arm
    public static final double kPivotArmSpeed = 0.2; //0.3; 
    //Sets the position for the arm when neutral
    public static final double kPivotArmNeutral = 0;
    

    public static final double pivotLowStop = 40;
    public static final double pivotHighStop = 85;

    public static final double kAnglesToTicks = 0;
    //Angles for scoring cones
    public static final double kHighAngleCone = 100.876; //(Actual)
    public static final double kMidAngleCone = 74.055; //(Actual)
    //Angle for scoring in the hybrid node common for cones and cubes
    public static final double kLowAngle = 41.601; //(actual)
    //Angles for scoring the cubes
    //Ofset to add 30 degrees
    public static final double kMidAngleCube = 90.7306362; // (Actual)
    public static final double kHighAngleCube = 96.7356393; // (Actual)


  
  }

  public static final class IOPorts{
    public static final int kDriverController = 1;
    public static final int kWeaponsContoller = 2;
  }

  public static final class DrivetrainConstants {
    public static final int LEFT_MOTOR2_ID = 2;
    public static final int LEFT_MOTOR_ID = 1;
    public static final int RIGHT_MOTOR2_ID = 7;
    public static final int RIGHT_MOTOR_ID = 6;
    public static final int STALL_LIMIT = 45;
    public static final int kDriverControllerPort = 1;
    public static final int kWeaponsControllerPort = 2;
    public static final String kShuffleboardTab = "Control Panel";
    public static final int kCurrentLimit = 60;
    public static final boolean kLeftReversedDefault = true;
    public static final boolean kRightReversedDefault = !kLeftReversedDefault;
    public static final double kTrackWidth = 29; // inches
    
    public static final double kS = 0.18; // minimum voltage to make the drivetrain move on the ground
    // driver encoder calculations
    // since the encoder is build into the motor we need to account for gearing
    public static final double kWheelDiameterInches = 6.0;
    private static final double kInitialGear = 14.0 / 58.0 * 18.0 / 38.0;
    private static final double kHighGear = kInitialGear * 32.0 / 34.0;
    private static final double kLowGear = kInitialGear * 22.0 / 44.0;

    // all measurements are based on inches and seconds
    public static final double kHighDistancePerPulse = kWheelDiameterInches * Math.PI * kHighGear;
    public static final double kHighSpeedPerPulse = kHighDistancePerPulse / 60.0;
    public static final double kLowDistancePerPulse = kWheelDiameterInches * Math.PI * kLowGear;
    public static final double kLowSpeedPerPulse = kLowDistancePerPulse / 60.0;

    // experimentally determined (inches per encoder count)
    public static final double kEncoderDistanceRatio = 1.125753635;
    public static double kRightAngleTurnArcLength = 7.25 * Math.PI;
    public static final double kHighSpeedPerPulseEncoderRatio = kEncoderDistanceRatio / 60.0;
    public static final double kControllerDeadzone = 0.1;
  }


	public static final class BalanceConstants{
		public static final double kBalancedBeamAngle = 0;
		public static final double kBalancedThreshold = 1;
		public static double kP = .018;
		public static double kI = 0.001;
		public static double kD = 0.00;
	}

	// PID Control (all experimentally determined)
	public static final class AutoForwardPIDValues{
		public static final double kP = 0.0576;
		public static final double kI = 0;
		public static final double kD = 0;	
		public static final double kVelocityTolerance = 5.0;
		public static final double kPositionTolerace = 0;
	}
	
	public static final class TurnDistanceGyroPID{
		public static final double kP = 0.029;
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kAngleTolerace = 4.0;
		public static final double kTurnSpeedTolerance = 1.0;
	}
 
	public static final class IntakeConstants {
    public static final int kIntakeMotorPort = 1;
    public static final int kIntakeRaiseChannel = 2;
    public static final int kIntakeLowerChannel = 3;
    public static final double kIntakeMotorSpeed = 10;
    public static final double kEjectMotorSpeed = -10;
    public static final Value kIntakeRaiseValue = Value.kForward;
    public static final Value kIntakeLowerValue = Value.kReverse;
    }

  public static final class VisionConstants {
    public static final double kCAMERA_HEIGHT = 0.0;
    public static final double kTARGET_HEIGHT = 0.0;
    public static final double kCAMERA_PITCH = 0.0;
  }

	public static class GripperConstants {
    // Most likely only be using one motor, but written code for 2 in case.
    public static final int kGripperRightMotor = 9;
    public static final int kGripperLeftMotor = 5;
    // Variable assigned values can change depending on what is needed for the robot.
    public static final double kGripperIntakeMotorSpeed = 0.45;   
    public static final double kGripperEjectMotorSpeed = -0.1;
    public static final int kBeambreak = 1;
    }

    public static class LightConstants {
        public static final String kShuffleboardTab = "Lights";
        public static final int kBlinkinDriverPort = 4; //TODO: Find a port for this
	  	  public static final double kDisabled = 0.61; //TODO: Find what color we want for this and its value
		    public static final double kLightsOff = 0.99;
        public static final double kRedBall = 0.67;
        public static final double kBlueBall = 0.87;
        public static final double kPurpleCube = 0.91;
        public static final double kYellowCone = 0.67;
        public static final double kDefaultColor = 0.93; //TODO: Find what we want default to be (same as disabled?)
        public static final double kParty = -0.99;
    }

	public static class ControlPanelConstants {
		public static final String kShuffleboardTab = "Control Panel";
	}
}