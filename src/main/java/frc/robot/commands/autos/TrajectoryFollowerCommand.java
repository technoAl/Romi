package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryFollowerCommand extends RamseteCommand{

DrivetrainSubsystem drivetrainSubsystem;

  public TrajectoryFollowerCommand(RobotContainer robotContainer, Trajectory trajectory) {
    super(trajectory,
      robotContainer.drivetrainSubsystem::getPose,
      new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
      AutoConstants.DRIVE_FEED_FORWARD,
      AutoConstants.DRIVE_KINEMATICS,
      robotContainer.drivetrainSubsystem::getWheelSpeeds,
      new PIDController(AutoConstants.P_DRIVE_VEL, 0, 0),
      new PIDController(AutoConstants.P_DRIVE_VEL, 0, 0),
      robotContainer.drivetrainSubsystem::tankDriveVolts,
      robotContainer.drivetrainSubsystem);
    this.drivetrainSubsystem = robotContainer.drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    drivetrainSubsystem.resetEncoder();
    drivetrainSubsystem.resetGyro();
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrainSubsystem.drive(0,0);
  }

}