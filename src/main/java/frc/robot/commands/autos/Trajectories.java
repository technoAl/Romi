package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import frc.robot.Constants.AutoConstants;

public class Trajectories {
    public final static TrajectoryConfig FORWARD_CONFIG = createTrajectoryConfig(false);
    public final static TrajectoryConfig BACKWARD_CONFIG = createTrajectoryConfig(true);

    private static TrajectoryConfig createTrajectoryConfig(boolean reversed) {
        final TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        config.setKinematics(AutoConstants.DRIVE_KINEMATICS);
        config.addConstraint(AutoConstants.AUTO_VOLTAGE_CONSTRAINT);
        config.setReversed(reversed);
        return config;
    }

}
