package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.RobotContainer;

public class DriveCommand extends CommandBase{
    
    DrivetrainSubsystem drivetrainSubsystem;

    public DriveCommand(RobotContainer robotContainer){
        drivetrainSubsystem = robotContainer.drivetrainSubsystem;
    }

    @Override
    public void initialize(){
        drivetrainSubsystem.resetEncoder();
        drivetrainSubsystem.resetGyro();
    }

    @Override
    public void execute(){
        drivetrainSubsystem.turn(180);
    }

    @Override
    public void end(boolean interrupted){
        drivetrainSubsystem.drive(0,0);
    }

}