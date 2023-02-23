package org.recordrobotics.charger.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;

public class Trajectories {

    public static TrajectoryConfig config = new TrajectoryConfig(2, 1); //DEFINE MAX VELOCITY AND ACCELERATION HERE

    public static Trajectory getTrajectory(Pose2d start, TrajectoryConfig config) {
        Rotation2d ballAngle = new Rotation2d(0);
        Rotation2d scoreAngle = new Rotation2d(Math.PI);
        Pose2d ball = new Pose2d(Units.inchesToMeters(277.949), Units.inchesToMeters(36.000), ballAngle);
        Pose2d score = new Pose2d(Units.inchesToMeters(69.438), Units.inchesToMeters(42.000), scoreAngle);
    
        ArrayList<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(start);
        waypoints.add(ball);
        waypoints.add(score);
    
        return TrajectoryGenerator.generateTrajectory(waypoints, config);
        }
}
