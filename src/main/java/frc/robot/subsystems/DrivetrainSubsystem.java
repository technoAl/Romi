package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.RomiGyro;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase{
    private Spark rightMotor, leftMotor;
    private Encoder rightEncoder, leftEncoder;
    private PIDController pid, pid2;
    private RomiGyro romiGyro;

    public DrivetrainSubsystem() {
        rightMotor = new Spark(DrivetrainConstants.RIGHT_MOTOR_PORT);
        leftMotor = new Spark(DrivetrainConstants.LEFT_MOTOR_PORT);
        rightMotor.setInverted(true);
        rightEncoder = new Encoder(DrivetrainConstants.RIGHT_ENCODER_A, DrivetrainConstants.RIGHT_ENCODER_B);
        leftEncoder = new Encoder(DrivetrainConstants.LEFT_ENCODER_A, DrivetrainConstants.LEFT_ENCODER_B);
        pid = new PIDController(0.001, 0, 0);
        pid2 = new PIDController(0.005, 0,0 );
        pid.setTolerance(5, 10);
        romiGyro = new RomiGyro();
    }

    @Override
    public void periodic() {
    
    }
    
    public void driveForward(double distance){
        double pulse = distance * (1/DrivetrainConstants.INCHES_PER_PULSE);
        rightMotor.set(pid.calculate(rightEncoder.getDistance(), pulse));
        leftMotor.set(pid.calculate(leftEncoder.getDistance(), pulse));
    }


    public void turn(double heading){ // in degrees
        rightMotor.set(pid2.calculate(romiGyro.getAngleZ(), heading));
        leftMotor.set(-pid2.calculate(romiGyro.getAngleZ(), heading));
    }

    public void drive(double left, double right){
        rightMotor.setSpeed(right);
        leftMotor.setSpeed(left);
    }

    public void resetEncoder(){
        rightEncoder.reset();
        leftEncoder.reset();
    }

    public void resetGyro(){
        romiGyro.reset();
    }
}
