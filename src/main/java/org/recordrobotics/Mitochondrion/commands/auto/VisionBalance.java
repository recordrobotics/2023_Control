package org.recordrobotics.Mitochondrion.commands.auto;

import java.util.List;

import org.recordrobotics.Mitochondrion.subsystems.Drive;
import org.recordrobotics.Mitochondrion.subsystems.NavSensor;
import org.recordrobotics.Mitochondrion.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class VisionBalance extends SequentialCommandGroup {

private Pose2d _center;
private Translation2d _edge;
private DifferentialDrivePoseEstimator _estimator;
private Drive _drive;
private Vision _vision;
private NavSensor _nav;
private Trajectory _trajectory;
private List<Translation2d> _waypoints;
private double _kp;

    public VisionBalance(Drive drive, NavSensor nav, Vision vision, DifferentialDrivePoseEstimator estimator, RamseteController ramsete, DifferentialDriveKinematics kinematics){
        _center = new Pose2d(Units.inchesToMeters(153.0025), Units.inchesToMeters(108.015), new Rotation2d(Math.PI));
        _edge = new Translation2d(Units.inchesToMeters(114.0025), Units.inchesToMeters(108.015));
        _estimator = estimator;
        _drive = drive;
        _nav = nav;
        _vision = vision;
        _kp = 1;
        _waypoints.add(_edge);
        _trajectory = TrajectoryGenerator.generateTrajectory(_estimator.getEstimatedPosition(), _waypoints, _center, new TrajectoryConfig(1, 0.5));

        addCommands(
            //new RamseteCommand(_trajectory, estimator::getEstimatedPosition, ramsete, 
            //    new SimpleMotorFeedforward(-0.12215, 1.4629, 5.9068), kinematics, drive::getWheelSpeeds, new PIDController(_kp, 0, 0), 
            //    new PIDController(_kp, 0, 0), drive::tankDriveVolts, _drive, _nav, _vision),
            new ChargeStationBalance(_drive, _nav)
        );
    }
}