package org.recordrobotics.charger.commands.auto;

import org.recordrobotics.charger.subsystems.Drive;
import org.recordrobotics.charger.subsystems.NavSensor;
import org.recordrobotics.charger.subsystems.Vision;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import edu.wpi.first.math.util.Units;


import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VisionDrive extends CommandBase {
	private Vision _vision;
	private Drive _drive;

	private Trajectory _traj;
	private DifferentialDrivePoseEstimator _estimator;
	private NavSensor _nav;
	private int _target;
	private Timer _timer;
	private RamseteController _ramseteController;

	/**
	 *
	 * @param vision vision system
	 * @param drive drivetrain
	 * @param trajectories trajectories
	 * @param target 0 for april tag target, 1 for cubes
	 */
	public VisionDrive(Vision vision, Drive drive, Trajectory trajectory, DifferentialDrivePoseEstimator estimator, NavSensor nav, int target){

		_vision = vision;
		_drive = drive;
		_traj = trajectory;
		_estimator = estimator;
		_target = target;
		_nav = nav;

		//_ramseteController = new RamseteController(1, 0);
		_ramseteController = new RamseteController();

	}

	@Override
	public void initialize() {
		_timer = new Timer();
		_timer.start();
		_vision.camera.setPipelineIndex(_target);
	}

	@Override
	public void execute() {

		System.out.println("Executing visiondrive! Timer: " + Timer.getFPGATimestamp());

		try {
			if (Vision.checkForTarget(_vision.camera)){
				double[] globalPose = Vision.estimateGlobalPose(_vision.camera);
				Pose2d visPose = new Pose2d(globalPose[0], globalPose[1], new Rotation2d(globalPose[5]));
				_estimator.addVisionMeasurement(visPose, Timer.getFPGATimestamp());

				System.out.println("Vision measurement added at: " + globalPose[0] + " " + globalPose[1] + " " + globalPose[5]);

			}
		} catch (NullPointerException e) {

		}


		// Calculates angle measurements given encoder values
		Rotation2d nav_sensor_spoof = new Rotation2d(
			((-1*_drive.getRightEncoder()/1000)-(-1*_drive.getLeftEncoder()/1000))/(2*Units.inchesToMeters(11)));

		System.out.println("Nav sensor spoof angle: " + nav_sensor_spoof);

		_estimator.update(nav_sensor_spoof, -1*_drive.getLeftEncoder()/1000, -1*_drive.getRightEncoder()/1000);



		// Get the desired pose from the trajectory.
		
		var desiredPose = _traj.sample(Timer.getFPGATimestamp());

		desiredPose.velocityMetersPerSecond = 0.1;

		Pose2d pose = _estimator.getEstimatedPosition();
		System.out.println("Kalman filter pose: " + pose.getX() + ", " + pose.getY() + ", " + pose.getRotation().getRadians());
		System.out.println("Desired pose: " + desiredPose);

		// Get the reference chassis speeds from the Ramsete controller.
		var refChassisSpeeds = _ramseteController.calculate(pose, desiredPose);


		ChassisSpeeds adjustedspeeds = _ramseteController.calculate(pose, desiredPose);

		//System.out.println(refChassisSpeeds.vxMetersPerSecond+", "+refChassisSpeeds.omegaRadiansPerSecond);

		//var chassisSpeeds = ChassisSpeeds(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
	
		//DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(adjustedSpeeds);
		//DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);


		// Set the linear and angular speeds.

		System.out.println("adjusted speeds from ramsete x, y, radians: " + adjustedspeeds.vxMetersPerSecond + " " + adjustedspeeds.vyMetersPerSecond + " " + adjustedspeeds.omegaRadiansPerSecond);

		_drive.move(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
		//_drive.move(-0.5,0.0);

		
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() > 10*_traj.getTotalTimeSeconds();
	}

}
