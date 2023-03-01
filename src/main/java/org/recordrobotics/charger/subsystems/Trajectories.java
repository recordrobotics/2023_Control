
package org.recordrobotics.charger.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;

@SuppressWarnings({"PMD"})
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

	public static Trajectory testTrajectory(Pose2d start, TrajectoryConfig config){
		Rotation2d ballAngle = new Rotation2d(0);
		Rotation2d scoreAngle = new Rotation2d(Math.PI/2);
		Pose2d ball = new Pose2d(1.42743, 2.748026, ballAngle);
		Pose2d score = new Pose2d(1.42743, 3.048026, scoreAngle);

		ArrayList<Pose2d> waypoints = new ArrayList<>();
		waypoints.add(start);
		waypoints.add(ball);
		waypoints.add(score);
		return TrajectoryGenerator.generateTrajectory(waypoints, config);
	}

	public static Trajectory visTestTrajectory(Pose2d start, TrajectoryConfig config){
		Rotation2d Angle1 = new Rotation2d(Math.PI/2);
		Rotation2d Angle2 = new Rotation2d(0);
		Pose2d Pose1 = new Pose2d(1.32743, 2.748026, Angle1);
		Pose2d Pose2 = new Pose2d(1.32743, 3.248026, Angle2);

		ArrayList<Pose2d> waypoints = new ArrayList<>();
		waypoints.add(start);
		waypoints.add(Pose1);
		waypoints.add(Pose2);
		return TrajectoryGenerator.generateTrajectory(waypoints, config);
	}
}
