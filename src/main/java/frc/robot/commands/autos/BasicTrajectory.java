package frc.robot.commands.autos;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.commands.autos.Trajectories;

public class BasicTrajectory extends Trajectories {
    private static final Pose2d END_POINT = new Pose2d(1, 0, new Rotation2d(0));
    public static final Trajectory MOVE = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, new Rotation2d(0)), END_POINT), 
        FORWARD_CONFIG);
    
}
