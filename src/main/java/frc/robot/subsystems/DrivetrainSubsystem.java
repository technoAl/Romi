package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.RomiGyro;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;

public class DrivetrainSubsystem extends SubsystemBase{
    private Spark rightMotor, leftMotor;
    private Encoder rightEncoder, leftEncoder;
    private PIDController pid, pid2;
    private RomiGyro romiGyro;

    private final Field2d field2d = new Field2d();

    private final DifferentialDriveOdometry odometry;

    private final DifferentialDrive drive;

    public DrivetrainSubsystem() {
        rightMotor = new Spark(DrivetrainConstants.RIGHT_MOTOR_PORT);
        leftMotor = new Spark(DrivetrainConstants.LEFT_MOTOR_PORT);
        // rightMotor.setInverted(true);

        rightEncoder = new Encoder(DrivetrainConstants.RIGHT_ENCODER_A, DrivetrainConstants.RIGHT_ENCODER_B);
        leftEncoder = new Encoder(DrivetrainConstants.LEFT_ENCODER_A, DrivetrainConstants.LEFT_ENCODER_B);
        rightEncoder.setDistancePerPulse(DrivetrainConstants.METERS_PER_PULSE);
        leftEncoder.setDistancePerPulse(DrivetrainConstants.METERS_PER_PULSE);
        // rightEncoder.setReverseDirection(true);

        pid = new PIDController(0.001, 0, 0);
        pid2 = new PIDController(0.005, 0,0 );
        pid.setTolerance(5, 10);

        romiGyro = new RomiGyro();
        drive = new DifferentialDrive(leftMotor, rightMotor);
        odometry = new DifferentialDriveOdometry(romiGyro.getRotation2d());
        SmartDashboard.putData("field", field2d);
        resetEncoder();
        resetGyro();
    }

    @Override
    public void periodic() {
        odometry.update(romiGyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

        field2d.setRobotPose(getPose());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
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

    public double getGyroAngleZ(){
        return romiGyro.getAngleZ();
    }

    public double getTurnRate() {
        return romiGyro.getRateZ();
      }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
      }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotor.setVoltage(leftVolts);
        rightMotor.setVoltage(-rightVolts);
        drive.feed();
    }

    public void setMaxOutput(double maxOutput) {
       drive.setMaxOutput(maxOutput);
      }

    public double getHeading() {
        return romiGyro.getRotation2d().getDegrees();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoder();
        odometry.resetPosition(pose, romiGyro.getRotation2d());
    }

    
}
