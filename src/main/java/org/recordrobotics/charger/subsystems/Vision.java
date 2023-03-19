package org.recordrobotics.charger.subsystems;

//import org.apache.commons.collections4.functors.TransformerClosure;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
	
	/*static double[][] tags = {
		{15.513558, 1.071626, 0.462788, Math.PI}, //tag 1
		{15.513558, 2.748026, 0.462788, Math.PI}, //tag 2
		{15.513558, 4.424426, 0.462788, Math.PI}, //tag 3
		{16.178784, 6.749796, 0.695452, Math.PI}, //tag 4
		{0.36195, 6.749796, 0.695452, 0}, //tag 5
		{1.02743, 4.424426, 0.462788, 0}, //tag 6
		{1.02743, 2.748026, 0.462788, 0}, //tag 7
		{1.02743, 1.071626, 0.462788, 0}}; //tag 8
	*/

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

		// Gets a frame from the camera
		var result = camera.getLatestResult();

			// Gets target object from apriltag perspective photonvision
			PhotonTrackedTarget target = result.getBestTarget();

			//System.out.println("Best Camera to target: " + target.getBestCameraToTarget());

			Transform3d robot_to_april = target.getBestCameraToTarget()/*.plus(robotToCam.inverse())*/; // you could put the offset here if you were testing for reals
			
			//System.out.println(robot_to_april.getRotation().getQuaternion());
			
			double w = robot_to_april.getRotation().getQuaternion().getW();
			double x = robot_to_april.getRotation().getQuaternion().getX();
			double y = robot_to_april.getRotation().getQuaternion().getY();
			double z = robot_to_april.getRotation().getQuaternion().getZ();
			
			double roll  = Math.atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z);
			double pitch = Math.atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z);
			double yaw   =  Math.asin(2*x*y + 2*z*w);

			double robot_to_april_x = robot_to_april.getX();
			double robot_to_april_y = robot_to_april.getY();
			double robot_to_april_z = robot_to_april.getZ();


			// if camera is upside down, rotates it. Comment out the below code if the camera is not upside down
			roll = (roll + 180) % 360;
			pitch = -1*pitch;
			yaw = -1*yaw;
			robot_to_april_y = -1*robot_to_april_y;
			robot_to_april_z = -1*robot_to_april_z;
			// End of upside down transformations


			//Creates a new transform 3d object
			Transform3d robot_to_april_transform3d = new Transform3d(new Translation3d(robot_to_april_x, robot_to_april_y, robot_to_april_z), new Rotation3d(roll, pitch, yaw));

			// Gets april to robot
			Transform3d april_to_robot = robot_to_april_transform3d.inverse();
			Pose3d april_to_robot_pose3d = new Pose3d(april_to_robot.getTranslation(), april_to_robot.getRotation());
			

			// Gets the fiducial ID and uses it to get the correct transform 2d object, which it then inverses to get the april to global perspective
			int targetID = target.getFiducialId();
			//Transform2d april_to_global = tag_transforms[targetID - 1].inverse();

			Transform3d april_to_global = tags[targetID - 1].inverse();
			//Pose3d global_to_april_pose3d = new Pose3d(tags[targetID - 1].getTranslation(),tags[targetID - 1].getRotation());
			Pose3d april_to_global_pose3d = new Pose3d(april_to_global.getTranslation(), april_to_global.getRotation());

			// finds difference
			Transform3d global_to_camera = new Transform3d(april_to_global_pose3d, april_to_robot_pose3d);

			System.out.println(global_to_camera);


			
			// Uses the "Transform2d(Pose2d initial, Pose2d final)" feature to take the difference between april to robot and april to global
			//Pose2d global_to_camera = april_to_robot_pose2d.plus(april_to_global);

			// Gets the X and Y or the transform
			double global_x = global_to_camera.getX();
			double global_y = global_to_camera.getY();
			double global_z = global_to_camera.getZ();
			double global_theta = global_to_camera.getRotation().toRotation2d().getRadians();
			double pitch_relative_to_apriltag = pitch;

			//SmartDashboard.putNumber("Tag ID", result.getBestTarget().getFiducialId());
			//SmartDashboard.putNumber("X value", result.getBestTarget().getBestCameraToTarget().getX());
			//SmartDashboard.putNumber("Y value", result.getBestTarget().getBestCameraToTarget().getY());
			//SmartDashboard.putNumber("Z value", result.getBestTarget().getBestCameraToTarget().getZ());
			//SmartDashboard.putNumber("Angle (degrees)", result.getBestTarget().getBestCameraToTarget().getRotation().toRotation2d().getDegrees());


			// Returns final global pose
			//System.out.println("POSE (x, y, radians): (" + global_x + ", " + global_y + ", " + global_theta + ")");
			return new double[] {global_x, global_y, global_theta}; // If this line doesnt work create a double[] called "global_pose" and return that instead
		
		}


	public static boolean checkForTarget(PhotonCamera camera){
		var result = camera.getLatestResult();//get a frame from the camera
		boolean hasTargets = result.hasTargets();//check for targets. This MUST be checked for, otherwise an error will occur if there isn't a target.
		return hasTargets;
	}


}
