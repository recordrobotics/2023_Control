package org.recordrobotics.charger;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;

@SuppressWarnings({"PMD"})

public class Pathfinding {

	TrajectoryConfig forward = new TrajectoryConfig(3, 1);
	TrajectoryConfig backward = new TrajectoryConfig(3, 1);

	char side;
	ArrayList<Pose2d> scoreSpots;
	Translation2d tLeft, tRight, bLeft, bRight;
	ArrayList<Pose2d> gamepieces;
	String routine, armDir;
	ArrayList<Trajectory> finalPath;

	public Pathfinding(Pose2d start, ArrayList<Pose2d> score, ArrayList<Pose2d> pieces, String r) {
		// config = c;
		double robotBuffer = 19;
		routine = r;

		// coordinates for what side we're on
		if(start.getX() < Units.inchesToMeters(325.16)) {
			side = 'B';
			tLeft = new Translation2d(Units.inchesToMeters(114.94-robotBuffer), Units.inchesToMeters(156.64+robotBuffer));
			tRight = new Translation2d(Units.inchesToMeters(191.065+robotBuffer), Units.inchesToMeters(156.64+robotBuffer));
			bLeft = new Translation2d(Units.inchesToMeters(114.94-robotBuffer), Units.inchesToMeters(59.39-robotBuffer));
			bRight = new Translation2d(Units.inchesToMeters(191.065+robotBuffer), Units.inchesToMeters(59.39-robotBuffer));
		} else {
			side = 'R';
			tLeft = new Translation2d(Units.inchesToMeters(459.255-robotBuffer), Units.inchesToMeters(156.64+robotBuffer));
			tRight = new Translation2d(Units.inchesToMeters(535.38+robotBuffer), Units.inchesToMeters(156.64+robotBuffer));
			bLeft = new Translation2d(Units.inchesToMeters(459.255-robotBuffer), Units.inchesToMeters(59.39-robotBuffer));
			bRight = new Translation2d(Units.inchesToMeters(535.38+robotBuffer), Units.inchesToMeters(59.39-robotBuffer));
		}

		scoreSpots = new ArrayList<>();
		for(Pose2d scoreSpot : score) {
			if(side == 'B') {
				scoreSpots.add(new Pose2d(scoreSpot.getX(), scoreSpot.getY(), new Rotation2d(Math.PI)));
			} else {
				scoreSpots.add(new Pose2d(scoreSpot.getX(), scoreSpot.getY(), new Rotation2d(0)));
			}
		}

		gamepieces = pieces;

		if(r == "docking") {
			finalPath = docking(start);
		} else {
			finalPath = scoring(start);
		}

		backward.setReversed(true);

	}

	public ArrayList<Trajectory> docking(Pose2d start) {
		ArrayList<Pose2d> path1 = new ArrayList<>();
		ArrayList<Pose2d> path2 = new ArrayList<>();
		ArrayList<Trajectory> trajectories = new ArrayList<>();
		String dir = "left";

		Pose2d dock = new Pose2d(Units.inchesToMeters(153.0025), Units.inchesToMeters(108.015), new Rotation2d(Math.PI));
		if(side == 'R') {
			dir = "right";
			dock = new Pose2d(Units.inchesToMeters(497.3175), Units.inchesToMeters(108.015), new Rotation2d(0));
		}

		path1.add(start);

		Pose2d score = bestScoring(start, dock);

		Pose2d[] intermediate = checkIntersection(score, start, dir);
		for(int i = 0; i < 2; i++) {
			if(intermediate[i] == null) break;
			path1.add(intermediate[i]);
		}

		double buffer = 21;

		if(side == 'B') {
			path1.add(adjustPose(score, Units.inchesToMeters(buffer), false));
			path2.add(adjustPose(score, Units.inchesToMeters(buffer), false));
		} else {
			path1.add(adjustPose(score, Units.inchesToMeters(buffer), true));
			path2.add(adjustPose(score, Units.inchesToMeters(buffer), true));
		}

		trajectories.add(TrajectoryGenerator.generateTrajectory(path1, forward));

		path2.add(dock);
		trajectories.add(TrajectoryGenerator.generateTrajectory(path2, backward));

		for(Pose2d p: path1) {
			System.out.println(p.getX() + " " + p.getY() + " " + p.getRotation().getRadians());
		}
		for(Pose2d p: path2) {
			System.out.println(p.getX() + " " + p.getY() + " " + p.getRotation().getRadians());
		}

		return trajectories;

	}

	public Pose2d bestScoring(Pose2d start, Pose2d dock) {
		ArrayList<Double> distances = new ArrayList<>();
		for(int i = 0; i < scoreSpots.size(); i++) {
			Pose2d score = scoreSpots.get(i);
			double startToScore = getDistance(start, score);
			double scoreToDock = getDistance(score, dock);
			distances.add(startToScore + scoreToDock);
		}

		Pose2d ret = scoreSpots.get(0);
		double retDist = distances.get(0);

		for(int i = 1; i < distances.size(); i++) {
			if(distances.get(i) < retDist) {
				retDist = distances.get(i);
				ret = scoreSpots.get(i);
			}
		}

		return ret;

	}

	public Pose2d[] checkIntersection(Pose2d a, Pose2d b, String dir) {
		if(a.getX() > b.getX()) {
			Pose2d temp = a;
			a = b;
			b = temp;
		}

		Pose2d[] ret = new Pose2d[2];

		double slope = (b.getY()-a.getY()) / (b.getX()-a.getX());
		double yIntercept = a.getY() - slope*a.getX();
		double leftY = slope*bLeft.getX() + yIntercept;
		double rightY = slope*tRight.getX() + yIntercept;

		if(b.getX() < bLeft.getX() || a.getX() > bRight.getX() || a.getY()>tLeft.getY() && b.getY()>tLeft.getY() || a.getY()<bLeft.getY() && b.getY()<bLeft.getY()) {
			leftY = 0;
			rightY = 0;
		}

		boolean leftIntersects = doesIntersect(leftY);
		boolean rightIntersects = doesIntersect(rightY);

		// System.out.println(leftIntersects + " " + rightIntersects );

		if(leftIntersects && rightIntersects) {
			double dist1 = linearDistance(a.getX(), a.getY(), tLeft.getX(), tLeft.getY()) + linearDistance(tLeft.getX(), tLeft.getY(), tRight.getX(), tRight.getY()) + linearDistance(tRight.getX(), tRight.getY(), b.getX(), b.getY());
			double dist2 = linearDistance(a.getX(), a.getY(), bLeft.getX(), bLeft.getY()) + linearDistance(bLeft.getX(), bLeft.getY(), bRight.getX(), bRight.getY()) + linearDistance(bRight.getX(), bRight.getY(), b.getX(), b.getY());

			if(dist1 < dist2) {

				// do a and tRight intersect?
				slope = (tRight.getY()-a.getY()) / (tRight.getX()-a.getX());
				yIntercept = a.getY() - slope*a.getX();
				leftY = slope*tLeft.getX() + yIntercept;
				leftIntersects = doesIntersect(leftY);

				// do tLeft and b intersect?
				slope = (tLeft.getY()-b.getY()) / (tLeft.getX()-b.getX());
				yIntercept = b.getY() - slope*b.getX();
				rightY = slope*tRight.getX() + yIntercept;
				rightIntersects = doesIntersect(rightY);

				if(leftIntersects && rightIntersects) {

					double angle = Math.atan( (tRight.getY()-a.getY()) / (tRight.getX()-a.getX()) );
					if(dir.equals("left")) angle += Math.PI;
					ret[0] = new Pose2d(tLeft.getX(), tLeft.getY(), new Rotation2d(angle));

					angle = Math.PI - Math.atan( (tLeft.getY()-b.getY()) / (b.getX() - tLeft.getX()) );
					if(dir.equals("right")) angle += Math.PI;
					ret[1] = new Pose2d(tRight.getX(), tRight.getY(), new Rotation2d(angle));

				} else if(leftIntersects) {

					double angle = Math.atan( (b.getY()-a.getY()) / (b.getX()-a.getX()) );
					if(dir.equals("left")) angle += Math.PI;
					ret[0] = new Pose2d(tLeft.getX(), tLeft.getY(), new Rotation2d(angle));

				} else if(rightIntersects) {

					double angle = Math.PI - Math.atan( (a.getY()-b.getY()) / (b.getX()-a.getX()) );
					if(dir.equals("right")) angle += Math.PI;
					ret[0] = new Pose2d(tRight.getX(), tRight.getY(), new Rotation2d(angle));

				}

			} else {

				// do a and bRight intersect?
				slope = (bRight.getY()-a.getY()) / (bRight.getX()-a.getX());
				yIntercept = a.getY() - slope*a.getX();
				leftY = slope*bLeft.getX() + yIntercept;
				leftIntersects = doesIntersect(leftY);

				// do bLeft and b intersect?
				slope = (bLeft.getY()-b.getY()) / (bLeft.getX()-b.getX());
				yIntercept = b.getY() - slope*b.getX();
				rightY = slope*bRight.getX() + yIntercept;
				rightIntersects = doesIntersect(rightY);

				// System.out.println(leftIntersects + " " + rightIntersects);

				if(leftIntersects && rightIntersects) {

					double angle = Math.PI - Math.atan( (a.getY()-bRight.getY()) / (bRight.getX() - a.getX()) );
					if(dir.equals("right")) angle += Math.PI;
					ret[0] = new Pose2d(bLeft.getX(), bLeft.getY(), new Rotation2d(angle));

					angle = Math.atan( (b.getY()-bLeft.getY()) / (b.getX()-bLeft.getX()) );
					if(dir.equals("left")) angle += Math.PI;
					ret[1] = new Pose2d(bRight.getX(), bRight.getY(), new Rotation2d(angle));

				} else if(leftIntersects) {

					double angle = Math.PI - Math.atan( (a.getY()-b.getY()) / (b.getX()-a.getX()) );
					if(dir.equals("right")) angle += Math.PI;
					System.out.println(angle);
					ret[0] = new Pose2d(bLeft.getX(), bLeft.getY(), new Rotation2d(angle));

				} else if(rightIntersects) {

					double angle = Math.atan( (b.getY()-a.getY()) / (b.getX()-a.getX()) );
					if(dir.equals("left")) angle += Math.PI;
					ret[0] = new Pose2d(bRight.getX(), bRight.getY(), new Rotation2d(angle));

				}
			}

		} else if(leftIntersects) {
			if(b.getY() > a.getY()) {
				double angle = Math.atan( (b.getY()-a.getY()) / (b.getX()-a.getX()) );
				if(dir.equals("left")) angle += Math.PI;
				ret[0] = new Pose2d(tLeft.getX(), tLeft.getY(), new Rotation2d(angle));
			} else {
				double angle = Math.PI - Math.atan( (a.getY()-b.getY()) / (b.getX() - a.getX()) );
				if(dir.equals("right")) angle += Math.PI;
				ret[0] = new Pose2d(bLeft.getX(), bLeft.getY(), new Rotation2d(angle));
			}
		} else if(rightIntersects) {
			if(b.getY() > a.getY()) {
				double angle = Math.atan( (b.getY()-a.getY()) / (b.getX()-a.getX()) );
				if(dir.equals("left")) angle += Math.PI;
				ret[0] = new Pose2d(bRight.getX(), bRight.getY(), new Rotation2d(angle));
			} else {
				double angle = Math.PI - Math.atan( (a.getY()-b.getY()) / (b.getX()-a.getX()) );
				if(dir.equals("right")) angle += Math.PI;
				ret[0] = new Pose2d(tRight.getX(), tRight.getY(), new Rotation2d(angle));
			}
		}

		return ret;
	}

	public boolean doesIntersect(double y) {
		return y >= bLeft.getY() && y <= tLeft.getY();
	}

	public double getDistance(Pose2d a, Pose2d b) {
		if(a.getX() > b.getX()) {
			Pose2d temp = a;
			a = b;
			b = temp;
		}

		double slope = (b.getY()-a.getY()) / (b.getX()-a.getX());
		double yIntercept = a.getY() - slope*a.getX();
		double leftY = slope*bLeft.getX() + yIntercept;
		double rightY = slope*tRight.getX() + yIntercept;

		if(b.getX() < bLeft.getX() || a.getX() > bRight.getX() || a.getY()>tLeft.getY() && b.getY()>tLeft.getY() || a.getY()<bLeft.getY() && b.getY()<bLeft.getY()) {
			leftY = 0;
			rightY = 0;
		}

		boolean leftIntersects = doesIntersect(leftY);
		boolean rightIntersects = doesIntersect(rightY);

		if(leftIntersects && rightIntersects) {
			double dist1 = linearDistance(a.getX(), a.getY(), tLeft.getX(), tLeft.getY()) + linearDistance(tLeft.getX(), tLeft.getY(), tRight.getX(), tRight.getY()) + linearDistance(tRight.getX(), tRight.getY(), b.getX(), b.getY());
			double dist2 = linearDistance(a.getX(), a.getY(), bLeft.getX(), bLeft.getY()) + linearDistance(bLeft.getX(), bLeft.getY(), bRight.getX(), bRight.getY()) + linearDistance(bRight.getX(), bRight.getY(), b.getX(), b.getY());
			return Math.min(dist1, dist2);
		} else if(leftIntersects) {
			if(b.getY() < a.getY()) {
				return linearDistance(a.getX(), a.getY(), tLeft.getX(), tLeft.getY()) + linearDistance(tLeft.getX(), tLeft.getY(), b.getX(), b.getY());
			} else {
				return linearDistance(a.getX(), a.getY(), bLeft.getX(), bLeft.getY()) + linearDistance(bLeft.getX(), bLeft.getY(), b.getX(), b.getY());
			}
		} else if(rightIntersects) {
			if(b.getY() > a.getY()) {
				return linearDistance(a.getX(), a.getY(), bRight.getX(), bRight.getY()) + linearDistance(bRight.getX(), bRight.getY(), b.getX(), b.getY());
			} else {
				return linearDistance(a.getX(), a.getY(), tRight.getX(), tRight.getY()) + linearDistance(tRight.getX(), tRight.getY(), b.getX(), b.getY());
			}
		} else {
			return linearDistance(a.getX(), a.getY(), b.getX(), b.getY());
		}
	}

	public double linearDistance(double x1, double y1, double x2, double y2) {
		double h = Math.abs(x1-x2);
		double v = Math.abs(y1-y2);
		return Math.sqrt((v*v)+(h*h));
	}

	// make a scoring path that scores and picks up three game pieces
	public ArrayList<Trajectory> scoring(Pose2d start) {
		ArrayList<Pose2d> origPath = new ArrayList<>();
		ArrayList<Pose2d> path = new ArrayList<>();

		ArrayList<Pose2d> scoreSpots2 = new ArrayList<>();
		scoreSpots2.addAll(scoreSpots);
		ArrayList<Pose2d> gamepieces2 = new ArrayList<>();
		gamepieces2.addAll(gamepieces);

		origPath.add(start);
		Pose2d prev = start;

		// pick order of points
		for(int i = 0; i < 3 && !scoreSpots2.isEmpty() && !gamepieces2.isEmpty(); i++) {
			// pick a scoring
			Pose2d bestScoring = scoreSpots2.get(0);
			double minDist = getDistance(prev, bestScoring);

			for(int j = 1; j < scoreSpots2.size(); j++) {
				double curDist = getDistance(prev, scoreSpots2.get(j));
				if(curDist < minDist) {
					bestScoring = scoreSpots2.get(j);
					minDist = curDist;
				}
			}

			scoreSpots2.remove(bestScoring);
			origPath.add(bestScoring);

			prev = bestScoring;

			// pick a piece
			Pose2d bestPiece = gamepieces2.get(0);
			minDist = getDistance(prev, bestPiece);

			for(int j = 1; j < gamepieces2.size(); j++) {
				double curDist = getDistance(prev, gamepieces2.get(j));
				if(curDist < minDist) {
					bestPiece = gamepieces2.get(j);
					minDist = curDist;
				}
			}

			gamepieces2.remove(bestPiece);
			origPath.add(bestPiece);
			prev = bestPiece;
		}

		// adjust path
		for(int i = 1; i < origPath.size(); i++) {
			Pose2d cur = origPath.get(i);

			if(side == 'B') {
				if(cur.getX() < tLeft.getX()) { //score
					double buffer = 21;
					cur = adjustPose(cur, Units.inchesToMeters(buffer), false);
					origPath.set(i, cur);
				} else if(cur.getX() > tRight.getX()) { // pick up
					double buffer = 21;
					Rotation2d angle = getAngle(side);
					cur = new Pose2d(cur.getX(), cur.getY(), angle);
					cur = adjustPose(cur, Units.inchesToMeters(buffer), true);
					origPath.set(i, cur);
				}
			} else if(side == 'R') {
				if(cur.getX() > tRight.getX()) { //score
					double buffer = 21;
					cur = adjustPose(cur, Units.inchesToMeters(buffer), true);
					origPath.set(i, cur);
				} else if(cur.getX() < tLeft.getX()) { // pick up
					double buffer = 21;
					Rotation2d angle = getAngle(side);
					cur = new Pose2d(cur.getX(), cur.getY(), angle);
					cur = adjustPose(cur, Units.inchesToMeters(buffer), false);
					origPath.set(i, cur);
				}
			}

		}

		printWaypoints(origPath);

		// add intermediate points
		boolean reverse = false;
		path.add(origPath.get(0));
		for(int i = 1; i < origPath.size(); i++) {
			String dir = "";

			if(side == 'B') dir = "left";
			else dir = "right";

			Pose2d[] intermediate = checkIntersection(origPath.get(i), origPath.get(i-1), dir);
			if(side == 'B' && !reverse || side == 'R' && reverse) {
				for(int k = 1; k >= 0; k--) {
					if(intermediate[k] != null) {
						path.add(intermediate[k]);
					}
				}
			} else {
				for(int k = 0; k < 2; k++) {
					if(intermediate[k] != null) {
						path.add(intermediate[k]);
					}
				}
			}

			reverse = !reverse;

			path.add(origPath.get(i));
		}

		printWaypoints(path);

		ArrayList<Trajectory> trajectories = new ArrayList<>();

		reverse = false;
		int begin = 0;

		for(int i = 1; i < path.size(); i++) {
			Pose2d cur = path.get(i);

			if(side == 'B') {
				if(!reverse) {
					if(cur.getX() > path.get(i-1).getX()) {
						trajectories.add(TrajectoryGenerator.generateTrajectory(path.subList(begin, i), forward));
						System.out.println(begin + " " + i + " " + reverse);
						begin = i-1;
						reverse = true;
					}
				} else {
					if(cur.getX() < path.get(i-1).getX()) {
						trajectories.add(TrajectoryGenerator.generateTrajectory(path.subList(begin, i), backward));
						System.out.println(begin + " " + i + " " + reverse);
						begin = i-1;
						reverse = false;
					}
				}

			} else {
				if(!reverse) {
					if(cur.getX() < path.get(i-1).getX()) {
						trajectories.add(TrajectoryGenerator.generateTrajectory(path.subList(begin, i), forward));
						System.out.println(begin + " " + i + " " + reverse);
						begin = i-1;
						reverse = true;
					}
				} else {
					if(cur.getX() > path.get(i-1).getX()) {
						trajectories.add(TrajectoryGenerator.generateTrajectory(path.subList(begin, i), backward));
						System.out.println(begin + " " + i + " " + reverse);
						begin = i-1;
						reverse = false;
					}
				}


			}
		}

		if(begin != path.size()-2) {
			if(!reverse) {
				trajectories.add(TrajectoryGenerator.generateTrajectory(path.subList(begin, path.size()), forward));
				System.out.println(begin + " " + path.size() + " " + reverse);
			} else {
				trajectories.add(TrajectoryGenerator.generateTrajectory(path.subList(begin, path.size()), backward));
				System.out.println(begin + " " + path.size() + " " + reverse);
			}
		}

		// concatenate trajectories?
		return trajectories;

	}

	public Rotation2d getAngle(char side) {
		if(side == 'B') {
			return new Rotation2d(Math.PI);
		} else {
			return new Rotation2d(0);
		}
	}

	// fix this for if robot picks up in two directions
	public Pose2d adjustPose(Pose2d p, double buffer, boolean rightSide) {
		double oldX = p.getX();
		double oldY = p.getY();
		double angle = p.getRotation().getRadians();

		double horizontal = Math.cos(angle)*buffer;
		double vertical = Math.sin(angle)*buffer;

		if(rightSide) return new Pose2d(oldX+horizontal, oldY+vertical, p.getRotation());

		return new Pose2d(oldX-horizontal, oldY-vertical, p.getRotation());

	}

	public void printWaypoints(ArrayList<Pose2d> path) {
		for(Pose2d p: path) {
			System.out.println(p.getX() + " " + p.getY() + " " + p.getRotation().getRadians());
		}
		System.out.println();
	}
}
