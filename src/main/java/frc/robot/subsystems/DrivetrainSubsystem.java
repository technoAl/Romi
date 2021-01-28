package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;;
import edu.wpi.first.wpilibj.Encoder;

public class DrivetrainSubsystem extends SubsystemBase{
    private Spark rightMotor, leftMotor;
    private Encoder rightEncoder, leftEncoder;
    private PIDController pid;

    public DrivetrainSubsystem() {
        rightMotor = new Spark(DrivetrainConstants.RIGHT_MOTOR_PORT);
        leftMotor = new Spark(DrivetrainConstants.LEFT_MOTOR_PORT);
        rightMotor.setInverted(true);
        rightEncoder = new Encoder(DrivetrainConstants.RIGHT_ENCODER_A, DrivetrainConstants.RIGHT_ENCODER_B);
        leftEncoder = new Encoder(DrivetrainConstants.LEFT_ENCODER_A, DrivetrainConstants.LEFT_ENCODER_B);
        pid = new PIDController(1, 0, 0);
    }

    @Override
    public void periodic() {
    
    }

    public void driveForward(double distance){
        rightMotor.set(pid.calculate(rightEncoder.getDistance(), distance));
        leftMotor.set(pid.calculate(leftEncoder.getDistance(), distance));
    }

    public void drive(double left, double right){
        rightMotor.setSpeed(right);
        leftMotor.setSpeed(left);
    }
}
