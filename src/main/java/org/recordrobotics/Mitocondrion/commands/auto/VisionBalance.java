package org.recordrobotics.Mitocondrion.commands.auto;

import org.recordrobotics.Mitocondrion.subsystems.Drive;
import org.recordrobotics.Mitocondrion.subsystems.NavSensor;
import org.recordrobotics.Mitocondrion.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionBalance extends SequentialCommandGroup {

private Pose2d _center;
private DifferentialDrivePoseEstimator _estimator;
private Drive _drive;
private Vision _vision;
private NavSensor _nav;

    public VisionBalance(Drive drive, NavSensor nav, Vision vision, DifferentialDrivePoseEstimator estimator){
        _center = new Pose2d(Units.inchesToMeters(153.0025), Units.inchesToMeters(108.015), new Rotation2d(Math.PI));
        _estimator = estimator;
        _drive = drive;
        _nav = nav;

        addCommands(
            new RamseteCommand(_center, estimator::getEstimatedPosition, ramsete, 
                new SimpleMotorFeedforward(0, 0), kinematics, drive::getWheelSpeeds, new PIDController(0, 0, 0), 
                new PIDController(0, 0, 0), drive::tankDriveVolts, drive, nav, vision),
            new ChargeStationBalance(_drive, _nav)
            )
    }

}