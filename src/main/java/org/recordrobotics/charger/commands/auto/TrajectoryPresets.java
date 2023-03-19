package org.recordrobotics.charger.commands.auto;

import java.util.ArrayList;

import org.recordrobotics.charger.subsystems.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;

public class TrajectoryPresets {

/*
 * quote from Lauren: 
 * Ok so basically, you want to make a Pathfinding object. 
 * If you look at the constructor, it takes in Pose2d start, ArrayLists for scoring nodes and game pieces, and a String for the routine we're executing. 
 * So pass in the robot's starting position, the list of poses of the possible scoring nodes that we're aiming for during the round, list of game piece poses, 
 * and then either "docking" (going onto the charge station) or "scoring" (continuously picking up and scoring pieces) for the routine. 
 * When the object is created, it will make a docking or scoring routine/trajectory for the robot that's stored in the variable finalPath. It's a list of trajectories.
 * docking: score the preloaded piece, dock at charge station
 * scoring: score repeatedly, only score
 */

    private Pathfinding _pathfinding;

    public ArrayList<Trajectory> blueBottomScoring(){
        Pose2d start = new Pose2d(1.74, 1.07, new Rotation2d(Math.PI));
        ArrayList<Pose2d> score = new ArrayList<Pose2d>();
        Pose2d score1 = new Pose2d(0.355704, 1.071626, new Rotation2d(Math.PI));
        Pose2d score2 = new Pose2d(0.800168, 1.071626, new Rotation2d(Math.PI));
        score.add(score1);
        score.add(score2);
        ArrayList<Pose2d> pieces = new ArrayList<Pose2d>();
        Pose2d piece1 = new Pose2d(7.06755, 0.919106, new Rotation2d(Math.PI));
        pieces.add(piece1);
        _pathfinding = new Pathfinding(start, score, pieces, "scoring");
        return _pathfinding.finalPath;
    }

    public ArrayList<Trajectory> blueMidScoring(){
        Pose2d start = new Pose2d(1.74, 2.75, new Rotation2d(Math.PI));
        ArrayList<Pose2d> score = new ArrayList<Pose2d>();
        Pose2d score1 = new Pose2d(0.355704, 2.74837, new Rotation2d(Math.PI));
        Pose2d score2 = new Pose2d(0.800168, 2.74837, new Rotation2d(Math.PI));
        score.add(score1);
        score.add(score2);
        ArrayList<Pose2d> pieces = new ArrayList<Pose2d>();
        Pose2d piece1 = new Pose2d(7.06755, 2.138426, new Rotation2d(Math.PI));
        pieces.add(piece1);
        _pathfinding = new Pathfinding(start, score, pieces, "scoring");
        return _pathfinding.finalPath;
    }

    public ArrayList<Trajectory> blueTopScoring(){
        Pose2d start = new Pose2d(1.74, 4.43, new Rotation2d(Math.PI));
        ArrayList<Pose2d> score = new ArrayList<Pose2d>();
        Pose2d score1 = new Pose2d(0.355704, 4.425099, new Rotation2d(Math.PI));
        Pose2d score2 = new Pose2d(0.800168, 4.425099, new Rotation2d(Math.PI));
        score.add(score1);
        score.add(score2);
        ArrayList<Pose2d> pieces = new ArrayList<Pose2d>();
        Pose2d piece1 = new Pose2d(7.06755, 4.576882, new Rotation2d(Math.PI));
        pieces.add(piece1);
        _pathfinding = new Pathfinding(start, score, pieces, "scoring");
        return _pathfinding.finalPath;
    }

    public ArrayList<Trajectory> redBottomScoring(){
        Pose2d start = new Pose2d(15.5, 1.07, new Rotation2d(0));
        ArrayList<Pose2d> score = new ArrayList<Pose2d>();
        Pose2d score1 = new Pose2d(16.185231, 1.071626, new Rotation2d(0));
        Pose2d score2 = new Pose2d(15.740733, 1.071626, new Rotation2d(0));
        score.add(score1);
        score.add(score2);
        ArrayList<Pose2d> pieces = new ArrayList<Pose2d>();
        Pose2d piece = new Pose2d(9.473502, 0.919106, new Rotation2d());
        pieces.add(piece);
        _pathfinding = new Pathfinding(start, score, pieces, "scoring");
        return _pathfinding.finalPath;
    }

    public ArrayList<Trajectory> redMidScoring(){
        Pose2d start = new Pose2d(15.5, 2.75, new Rotation2d(0));
        ArrayList<Pose2d> score = new ArrayList<Pose2d>();
        Pose2d score1 = new Pose2d(16.185231, 2.74837, new Rotation2d(0));
        Pose2d score2 = new Pose2d(15.740733, 2.74837, new Rotation2d(0));
        score.add(score1);
        score.add(score2);
        ArrayList<Pose2d> pieces = new ArrayList<Pose2d>();
        Pose2d piece1 = new Pose2d(9.473502, 2.138426, new Rotation2d(0));
        pieces.add(piece1);
        _pathfinding = new Pathfinding(start, score, pieces, "scoring");
        return _pathfinding.finalPath;
    }

    public ArrayList<Trajectory> redTopScoring(){
        Pose2d start = new Pose2d(15.5, 4.43, new Rotation2d(0));
        ArrayList<Pose2d> score = new ArrayList<Pose2d>();
        Pose2d score1 = new Pose2d(16.185231, 4.425099, new Rotation2d(0));
        Pose2d score2 = new Pose2d(15.740733, 4.425099, new Rotation2d(0));
        score.add(score1);
        score.add(score2);
        ArrayList<Pose2d> pieces = new ArrayList<Pose2d>();
        Pose2d piece1 = new Pose2d(9.473502, 4.576882, new Rotation2d(0));
        pieces.add(piece1);
        _pathfinding = new Pathfinding(start, score, pieces, "scoring");
        return _pathfinding.finalPath;
    }

    public ArrayList<Trajectory> blueBottomDocking(){
        Pose2d start = new Pose2d(1.74, 1.07, new Rotation2d(Math.PI));
        ArrayList<Pose2d> score = new ArrayList<Pose2d>();
        Pose2d score1 = new Pose2d(0.355704, 1.071626, new Rotation2d(Math.PI));
        score.add(score1);
        ArrayList<Pose2d> pieces = new ArrayList<Pose2d>();
        _pathfinding = new Pathfinding(start, score, pieces, "docking");
        return _pathfinding.finalPath;
    }

    public ArrayList<Trajectory> blueMidDocking(){
        Pose2d start = new Pose2d(1.74, 2.75, new Rotation2d(Math.PI));
        ArrayList<Pose2d> score = new ArrayList<Pose2d>();
        Pose2d score1 = new Pose2d(0.355704, 2.74837, new Rotation2d(Math.PI));
        score.add(score1);
        ArrayList<Pose2d> pieces = new ArrayList<Pose2d>();
        _pathfinding = new Pathfinding(start, score, pieces, "docking");
        return _pathfinding.finalPath;
    }

    public ArrayList<Trajectory> blueTopDocking(){
        Pose2d start = new Pose2d(1.74, 4.43, new Rotation2d(Math.PI));
        ArrayList<Pose2d> score = new ArrayList<Pose2d>();
        Pose2d score1 = new Pose2d(0.355704, 4.425099, new Rotation2d(Math.PI));
        score.add(score1);
        ArrayList<Pose2d> pieces = new ArrayList<Pose2d>();
        _pathfinding = new Pathfinding(start, score, pieces, "docking");
        return _pathfinding.finalPath;
    }

    public ArrayList<Trajectory> redBottomDocking(){
        Pose2d start = new Pose2d(15.5, 1.07, new Rotation2d(0));
        ArrayList<Pose2d> score = new ArrayList<Pose2d>();
        Pose2d score1 = new Pose2d(16.185231, 1.071626, new Rotation2d(0));
        score.add(score1);
        ArrayList<Pose2d> pieces = new ArrayList<Pose2d>();
        _pathfinding = new Pathfinding(start, score, pieces, "docking");
        return _pathfinding.finalPath;
    }

    public ArrayList<Trajectory> redMidDocking(){
        Pose2d start = new Pose2d(15.5, 2.75, new Rotation2d(0));
        ArrayList<Pose2d> score = new ArrayList<Pose2d>();
        Pose2d score1 = new Pose2d(16.185231, 2.74837, new Rotation2d(0));
        score.add(score1);
        ArrayList<Pose2d> pieces = new ArrayList<Pose2d>();
        _pathfinding = new Pathfinding(start, score, pieces, "docking");
        return _pathfinding.finalPath;
    }

    public ArrayList<Trajectory> redTopDocking(){
        Pose2d start = new Pose2d(15.5, 4.43, new Rotation2d(0));
        ArrayList<Pose2d> score = new ArrayList<Pose2d>();
        Pose2d score1 = new Pose2d(16.185231, 4.425099, new Rotation2d(0));
        score.add(score1);
        ArrayList<Pose2d> pieces = new ArrayList<Pose2d>();
        _pathfinding = new Pathfinding(start, score, pieces, "docking");
        return _pathfinding.finalPath;
    }

    public Trajectory blueNoScoring(){
        Pose2d start = new Pose2d(1.74, 2.75, new Rotation2d(Math.PI));//middle start, facing outwards
        Pose2d outOfCommunity = new Pose2d(6.22, 2.75, new Rotation2d(Math.PI));
        Pose2d dock = new Pose2d(Units.inchesToMeters(153.0025), Units.inchesToMeters(108.015), new Rotation2d(Math.PI));
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
        poses.add(start);
        poses.add(outOfCommunity);
        poses.add(dock);
        Trajectory traj = TrajectoryGenerator.generateTrajectory(poses, _pathfinding.forward);
        return traj;
    }

    public Trajectory redNoScoring(){
        Pose2d start = new Pose2d(15.5, 2.75, new Rotation2d(0));
        Pose2d outOfCommunity = new Pose2d(10.28, 2.75, new Rotation2d());
        Pose2d dock = new Pose2d(Units.inchesToMeters(497.3175), Units.inchesToMeters(108.015), new Rotation2d(0));
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
        poses.add(start);
        poses.add(outOfCommunity);
        poses.add(dock);
        Trajectory traj = TrajectoryGenerator.generateTrajectory(poses, _pathfinding.forward);
        return traj;
    }

    public Trajectory blueTaxiOnly(){
        Pose2d start = new Pose2d(2.54, 4.65, new Rotation2d(Math.PI));//middle start, facing outwards
            Pose2d outOfCommunity = new Pose2d(4.54, 4.65, new Rotation2d(Math.PI));
            ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
            poses.add(start);
            poses.add(outOfCommunity);
            Trajectory traj = TrajectoryGenerator.generateTrajectory(poses, _pathfinding.forward);
            return traj;
    }

    public Trajectory redTaxiOnly(){
        Pose2d start = new Pose2d(13.98, 4.65, new Rotation2d(0));
        Pose2d outOfCommunity = new Pose2d(11.98, 4.65, new Rotation2d(0));
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
        poses.add(start);
        poses.add(outOfCommunity);
        Trajectory traj = TrajectoryGenerator.generateTrajectory(poses, _pathfinding.forward);
        return traj;
    }

    public Trajectory testTraj(){
        Pose2d start = new Pose2d(2.54, 4.65, new Rotation2d(0));//middle start, facing outwards
            Pose2d outOfCommunity = new Pose2d(4.54, 4.65, new Rotation2d(0));
            ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
            poses.add(start);
            poses.add(outOfCommunity);
            _pathfinding = new Pathfinding(start, poses, poses, "string");
            Trajectory traj = TrajectoryGenerator.generateTrajectory(poses, _pathfinding.slooooowwww);
            return traj;
    }

    public ArrayList<Trajectory> SpinSpin9000(){
        Pose2d start = new Pose2d(2.54, 2.748, new Rotation2d(Math.PI));

        ArrayList<Pose2d> score = new ArrayList<Pose2d>();
        Pose2d score1 = new Pose2d(4, 2.74837, new Rotation2d(Math.PI));
        score.add(score1);
        
        ArrayList<Pose2d> pieces = new ArrayList<Pose2d>();
        Pose2d piece1 = new Pose2d(4,4, new Rotation2d(Math.PI));
        pieces.add(piece1);

        _pathfinding = new Pathfinding(start, score, pieces, "docking");
        return _pathfinding.finalPath;
    }
}