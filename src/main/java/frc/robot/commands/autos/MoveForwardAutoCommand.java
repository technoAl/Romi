package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.autos.BasicTrajectory;


public class MoveForwardAutoCommand extends SequentialCommandGroup {
  public MoveForwardAutoCommand(RobotContainer robotContainer) {
    addCommands(
       new TrajectoryFollowerCommand(robotContainer, BasicTrajectory.MOVE)
    );
  }
}