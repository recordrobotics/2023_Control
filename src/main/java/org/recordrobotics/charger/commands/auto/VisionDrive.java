package org.recordrobotics.charger.commands.auto;

import org.recordrobotics.charger.subsystems.Drive;
import org.recordrobotics.charger.subsystems.NavSensor;
import org.recordrobotics.charger.subsystems.Vision;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
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
	private final RamseteController _ramseteController = new RamseteController();

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
	}

	@Override
	public void initialize() {
		_timer = new Timer();
		_timer.start();
		_vision.camera.setPipelineIndex(_target);
	}

	@Override
	public void execute() {
		if (Vision.checkForTarget(_vision.camera)){
			double[] globalPose = Vision.estimateGlobalPose(_vision.camera);
			Pose2d visPose = new Pose2d(globalPose[0], globalPose[1], new Rotation2d(globalPose[2]));
			_estimator.addVisionMeasurement(visPose, _timer.get());
		}
		_estimator.update(new Rotation2d(_nav.getYaw()), _drive.getLeftEncoder()/1000, _drive.getRightEncoder()/1000);
		// Get the desired pose from the trajectory.
		var desiredPose = _traj.sample(_timer.getFPGATimestamp());
		Pose2d pose = _estimator.getEstimatedPosition();
		System.out.println(pose.getX() + ", " + pose.getY() + ", " + pose.getRotation().getRadians());
		System.out.println(desiredPose);

		// Get the reference chassis speeds from the Ramsete controller.
		var refChassisSpeeds = _ramseteController.calculate(pose, desiredPose);

		System.out.println(refChassisSpeeds.vxMetersPerSecond+", "+refChassisSpeeds.omegaRadiansPerSecond);
		// Set the linear and angular speeds.
		//_drive.move(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
		_drive.move(-0.5,0.0);
	}

	@Override
	public boolean isFinished() {
		return _timer.getFPGATimestamp() > 20*_traj.getTotalTimeSeconds();
	}

}
