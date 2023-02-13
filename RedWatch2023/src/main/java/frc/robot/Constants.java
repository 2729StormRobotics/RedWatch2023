// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  public static final int LEFT_MOTOR2_ID = 2;
  public static final int LEFT_MOTOR_ID = 1;
  public static final int RIGHT_MOTOR2_ID = 7;
  public static final int RIGHT_MOTOR_ID = 6;
	public static final int STALL_LIMIT = 45;
	public static final int kDriverControllerPort = 1;
	public static final String kShuffleboardTab = "Testing";
	public static final int kCurrentLimit = 60;
	public static final boolean kLeftReversedDefault = false;
	public static final boolean kRightReversedDefault = !kLeftReversedDefault;
	public static final double kTrackWidth = 29; // inches
	
	public static final double kS = 0.29; // minimum voltage to make the drivetrain move on the ground
	// driver encoder calculations
	// since the encoder is build into the motor we need to account for gearing
	public static final double kWheelDiameterInches = 6.0;
	private static final double kWheelDiameterFeet = kWheelDiameterInches / 12.0;
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
    public static final int kDriverController = 0;
	public static final double kControllerDeadzone = 0.1;

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

    public static final class HopperConstants{ 
        public static final int kHopperMotorPort = 5;
        public static final double kHopperMotorSpeed = 0.25; 

        public static final String kShuffleboardTab = "Testing";
    }

    public static final class ColorConstants {
        public static final int colorPort = 2;
        public static final String kShuffleboardTab = "Color";
    }
    public static final class VisionConstants {
        public static final double kCAMERA_HEIGHT = 0.0;
        public static final double kTARGET_HEIGHT = 0.0;
        public static final double kCAMERA_PITCH = 0.0;
    }

}
