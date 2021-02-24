package frc.robot.commands.autos;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.commands.autos.Trajectories;

public class BasicTrajectory extends Trajectories {
    private static final Pose2d END_POINT = new Pose2d(0.25, 0, new Rotation2d(0));
    public static final Trajectory MOVE = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(0.5, 0.25),
            new Translation2d(1.0, -0.25),
            new Translation2d(1.5, 0)
        ),
        new Pose2d(0.0, 0, new Rotation2d(Math.PI)),
        FORWARD_CONFIG);
    
}
