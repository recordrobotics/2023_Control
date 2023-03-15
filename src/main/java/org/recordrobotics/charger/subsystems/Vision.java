package org.recordrobotics.charger.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings({"PMD.SystemPrintln", "PMD.FieldNamingConventions"})
public class Vision extends SubsystemBase{

	public PhotonCamera camera = new PhotonCamera("OV5647"); //IMPORTANT: This camera name MUST match the one on the Raspberry Pi, accessible through the PhotonVision UI.
	public Transform3d robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(11), -1*Units.inchesToMeters(9), 0.1725), new Rotation3d(0,0,0)); //The offset from the center of the robot to the camera, and from facing exactly forward to the orientation of the camera.
	//TODO: SET TRANSFORM!!!!!!


	// Sends photonvision camera output to shuffleboard
	//public void runDriverMode() {
	//	camera.setDriverMode(true);
	//	camera.setPipelineIndex(2);
	//}

	static final Transform2d[] tag_transforms = {//april tags 1-8 in order. values contained are x, y, z, theta, in that order. x, y, z are distances in meters, theta is in radians.
		
		new Transform2d(new Translation2d(15.513558, 4.424426), new Rotation2d(Math.PI)), //tag 1
		new Transform2d(new Translation2d(15.513558, 2.748026), new Rotation2d(Math.PI)), //tag 2
		new Transform2d(new Translation2d(15.513558, 4.424426), new Rotation2d(Math.PI)), //tag 3
		new Transform2d(new Translation2d(16.178784, 6.749796), new Rotation2d(Math.PI)), //tag 4
		new Transform2d(new Translation2d(0.36195, 6.749796), new Rotation2d(0)), //tag 5
		new Transform2d(new Translation2d(1.02743, 4.424426), new Rotation2d(0)), //tag 6
		new Transform2d(new Translation2d(1.02743, 2.748026), new Rotation2d(0)), //tag 7
		new Transform2d(new Translation2d(1.02743, 1.071626), new Rotation2d(0)),}; //tag 8
	
	public static double[] estimateGlobalPose(PhotonCamera camera) {

		// Gets a frame from the camera
		var result = camera.getLatestResult();

		// Checks if there is a target. If there is one, the "if" statement passes. 
		boolean hasTargets = result.hasTargets();
		if (hasTargets){
			
			// Gets target object from apriltag perspective photonvision
			PhotonTrackedTarget target = result.getBestTarget();
			Transform3d robot_to_april = target.getBestCameraToTarget()/*.plus(robotToCam.inverse())*/; // you could put the offset here if you were testing for reals
			Transform3d april_to_robot = robot_to_april.inverse();
			// Converts the Transform3d object into a Pose2d object. Probably not entirely necessary to shift everything to 2d but its 2:30 in the morning and honestly my brain is kind of not working
			Pose2d april_to_robot_pose2d = new Pose2d(
				april_to_robot.getTranslation().toTranslation2d(), 
				april_to_robot.getRotation().toRotation2d());

			// Gets the fiducial ID and uses it to get the correct transform 2d object, which it then inverses to get the april to global perspective
			int targetID = target.getFiducialId();
			Transform2d april_to_global = tag_transforms[targetID - 1].inverse();

			//System.out.println(april_to_global);
			
			// Uses the "Transform2d(Pose2d initial, Pose2d final)" feature to take the difference between april to robot and april to global
			Pose2d global_to_camera = april_to_robot_pose2d.plus(april_to_global);

			// Gets the X and Y or the transform
			double global_x = global_to_camera.getX();
			double global_y = global_to_camera.getY();
			double global_theta = global_to_camera.getRotation().getRadians();

			// Returns final global pose
			return new double[] {global_x, global_y, global_theta}; // If this line doesnt work create a double[] called "global_pose" and return that instead
		}

		else {return null;}
	}

	public static boolean checkForTarget(PhotonCamera camera){
		var result = camera.getLatestResult();//get a frame from the camera
		boolean hasTargets = result.hasTargets();//check for targets. This MUST be checked for, otherwise an error will occur if there isn't a target.
		return hasTargets;
	}


}
