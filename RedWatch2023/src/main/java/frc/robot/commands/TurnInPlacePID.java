package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import static frc.robot.Constants.*;
import frc.robot.subsystems.Drivetrain;

public class TurnInPlacePID extends PIDCommand {

    private Drivetrain m_drivetrain;
    private double turnDeg;

    /**
     * Rotates the robot a given amount of degrees
     * @param turnDeg degrees to turn the robot
     * @Requirements Drivetrain
     */
    public TurnInPlacePID(double turnDeg, Drivetrain drivetrain) {

        super(
            new PIDController(TURN_kP, TURN_kI, TURN_kD),
            drivetrain::getPitch,
            drivetrain.getPitch()+ turnDeg, // setpoint initialized in initalize
            output -> drivetrain.tankDrive(output + Math.copySign(TURN_kF, output), -output - Math.copySign(TURN_kF, output), true),
            drivetrain
        );

        this.m_drivetrain = drivetrain;
        this.turnDeg = turnDeg;

        addRequirements(m_drivetrain);
        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(12);
    }

    @Override
    public void initialize() {
        
    m_drivetrain.ahrs.reset();
        super.initialize();

        // Hack to make sure the robot turns turnDeg degrees from current heading and not turnDeg degrees from 0
        double setpoint = MathUtil.inputModulus(m_drivetrain.getHeading() + turnDeg, -180, 180);
        m_setpoint = () ->  setpoint;
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}