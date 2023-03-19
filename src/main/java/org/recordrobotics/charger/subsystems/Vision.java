package org.recordrobotics.charger.subsystems;

//import org.apache.commons.collections4.functors.TransformerClosure;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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

	static Transform3d[] tags = {//april tags 1-8 in order. values contained are x, y, z, theta, in that order. x, y, z are distances in meters, theta is in radians.
		
		new Transform3d(new Translation3d(15.513558, 1.071626, 0.462788), new Rotation3d(0,0,Math.PI)),
		new Transform3d(new Translation3d(15.513558, 2.748026, 0.462788), new Rotation3d(0,0,Math.PI)),
		new Transform3d(new Translation3d(15.513558, 4.424426, 0.462788), new Rotation3d(0,0,Math.PI)),
		new Transform3d(new Translation3d(16.178784, 6.749796, 0.695452), new Rotation3d(0,0,Math.PI)),
		new Transform3d(new Translation3d(0.36195, 6.749796, 0.695452), new Rotation3d(0,0,0)),
		new Transform3d(new Translation3d(1.02743, 4.424426, 0.462788), new Rotation3d(0,0,0)),
		new Transform3d(new Translation3d(1.02743, 2.748026, 0.462788), new Rotation3d(0,0,0)),
		new Transform3d(new Translation3d(1.02743, 1.071626, 0.462788), new Rotation3d(0,0,0))
	};
	
	public static double[] estimateGlobalPose(PhotonCamera camera) {
		System.out.println("vision detected!");

		// Gets a frame from the camera
		var result = camera.getLatestResult();

			// Gets target object from apriltag perspective photonvision
			PhotonTrackedTarget target = result.getBestTarget();
			// Gets the Transform3d object for april to robot
			Transform3d robot_to_april = target.getBestCameraToTarget()/*.plus(robotToCam.inverse())*/; // you could put the offset here if you were testing for reals
			Transform3d april_to_robot = robot_to_april.inverse();
			// Gets the fiducial ID and uses it to get the correct transform 2d object
			int targetID = target.getFiducialId();
			Transform3d global_to_april = tags[targetID - 1];
			// Adds the two transform objects together to get robot to global
			Transform3d robot_to_global = global_to_april.plus(april_to_robot);

			
			// Calculates pitch, roll, and yaw
			double w = robot_to_april.getRotation().getQuaternion().getW();
			double x = robot_to_april.getRotation().getQuaternion().getX();
			double y = robot_to_april.getRotation().getQuaternion().getY();
			double z = robot_to_april.getRotation().getQuaternion().getZ();
			
			double roll  = Math.atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z);
			double pitch = Math.atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z);
			double yaw   = Math.asin(2*x*y + 2*z*w);


			//Puts the important stuff on the SmartDashboard
			SmartDashboard.putNumber("X value", robot_to_global.getX());
			SmartDashboard.putNumber("Y value", robot_to_global.getY());
			SmartDashboard.putNumber("Z value", robot_to_global.getZ());
			SmartDashboard.putNumber("Angle (degrees)", robot_to_global.getRotation().toRotation2d().getDegrees());


			// Returns final global pose
			return new double[] {
				robot_to_global.getX(), 
				robot_to_global.getY(), 
				robot_to_global.getZ(),
				pitch, roll, yaw}; 
		}


	public static boolean checkForTarget(PhotonCamera camera){
		var result = camera.getLatestResult();//get a frame from the camera
		boolean hasTargets = result.hasTargets();//check for targets. This MUST be checked for, otherwise an error will occur if there isn't a target.
		return hasTargets;
	}


}
