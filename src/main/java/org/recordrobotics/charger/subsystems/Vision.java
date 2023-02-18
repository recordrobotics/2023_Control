package org.recordrobotics.charger.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings({"PMD.SystemPrintln", "PMD.FieldNamingConventions"})
public class Vision extends SubsystemBase{

	public PhotonCamera camera = new PhotonCamera("OV5647"); //IMPORTANT: This camera name MUST match the one on the Raspberry Pi, accessible through the PhotonVision UI.
	public Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //The offset from the center of the robot to the camera, and from facing exactly forward to the orientation of the camera.

	static final double[][] tags = {//april tags 1-8 in order. values contained are x, y, z, theta, in that order. x, y, z are distances in meters, theta is in radians.
		{15.513558, 1.071626, 0.462788, Math.PI}, //tag 1
		{15.513558, 2.748026, 0.462788, Math.PI}, //tag 2
		{15.513558, 4.424426, 0.462788, Math.PI}, //tag 3
		{16.178784, 6.749796, 0.695452, Math.PI}, //tag 4
		{0.36195, 6.749796, 0.695452, 0}, //tag 5
		{1.02743, 4.424426, 0.462788, 0}, //tag 6
		{1.02743, 2.748026, 0.462788, 0}, //tag 7
		{1.02743, 1.071626, 0.462788, 0}}; //tag 8
	double[] fieldDimensions = {16.54175, 8.0137};//x, y. The origin is at the bottom corner of the blue alliance wall as seen on the field drawings. 0 radians is parallel to the positive x-axis. Distances are meters.

	public static double[] getVisionPoseEstimate(PhotonCamera camera, Transform3d robotToCam){
		var result = camera.getLatestResult();//get a frame from the camera
		boolean hasTargets = result.hasTargets();//check for targets. This MUST be checked for, otherwise an error will occur if there isn't a target.
		if (hasTargets){
			PhotonTrackedTarget target = result.getBestTarget();//Choose the closest Apriltag
			int targetID = target.getFiducialId();//get the ID
			Transform3d bestCameraToTarget = target.getBestCameraToTarget();//getting the transform from the camera to the tag. This transform's values are robot relative, and so will be converted to global coordinates.
			double yaw = target.getYaw();//getting the angle to the robot from the center of the april tag.
			double distance = Math.sqrt(bestCameraToTarget.getX()*bestCameraToTarget.getX() + bestCameraToTarget.getY()*bestCameraToTarget.getY());//getting the distance between the camera and tag from the x and y components of the transform.
			double x_transform = Math.cos(yaw)*distance;//getting the x transform from the tag to the robot.
			double y_transform = Math.sin(yaw)*distance;//getting the y transform from the tag to the robot.
			double global_x = tags[targetID][0] + x_transform;//the x transform from the tag to the robot is added to the tag's x coordinate to get the robot's global x coordinate.
			double global_y = tags[targetID][1] + y_transform;//the y transform from the tag to the robot is added to the tag's y coordinate to get the robot's global y coordinate.
			double global_theta = tags[targetID][3] + Math.PI + bestCameraToTarget.getRotation().toRotation2d().getRadians(); //getting the orientation of the robot from the tag's orientation and the transform. Pi is added because if the camera sees the tag, it is necessarily looking in the direction opposite the tag's orientation.
			double[] globalPose = {global_x, global_y, global_theta};//returns a [x, y, theta] vector. The z of the robot is irrelevant for pose estimation this year, and so ignored here.
			return globalPose;
		} else
		return null;//if no tag is visible, nothing will be returned.
	}

	public static double[] getColoredObjectPose(PhotonCamera camera, Transform3d robotToCam, DifferentialDrivePoseEstimator estimator){
		var result = camera.getLatestResult();//get a frame from the camera
		boolean hasTargets = result.hasTargets();//check for targets. This MUST be checked for, otherwise an error will occur if there isn't a target.
		if (hasTargets){
			PhotonTrackedTarget target = result.getBestTarget();//Choose the closest colored object of the type being looked for
			Transform3d bestCameraToTarget = target.getBestCameraToTarget();//getting the transform from the camera to the object. These coordinates are robot-relative, and so for trajectory following will be converted to global coordinates below.
			Pose2d robotPose = estimator.getEstimatedPosition();
			double distance = Math.sqrt(bestCameraToTarget.getX()*bestCameraToTarget.getX() + bestCameraToTarget.getY()*bestCameraToTarget.getY());//getting the distance between the camera and the object from the x and y components of the transform.
			double globalX = Math.cos(robotPose.getRotation().getRadians())*distance;
			double globalY = Math.sin(robotPose.getRotation().getRadians())*distance;
			double globalTheta = bestCameraToTarget.getRotation().toRotation2d().getRadians() + robotPose.getRotation().getRadians();
			double[] globalPose = {globalX, globalY, globalTheta};
			return globalPose;
		} else
		return null;
	}
}
