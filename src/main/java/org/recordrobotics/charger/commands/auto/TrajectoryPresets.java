package org.recordrobotics.charger.commands.auto;

import java.util.ArrayList;

import org.recordrobotics.charger.subsystems.Pathfinding;

import edu.wpi.first.math.trajectory.Trajectory;

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
        _pathfinding = new Pathfinding(null, null, null, "scoring");
        return _pathfinding.scoring(null);
    }

    public ArrayList<Trajectory> blueMidScoring(){
        _pathfinding = new Pathfinding(null, null, null, "scoring");
        return null;
    }

    public ArrayList<Trajectory> blueTopScoring(){
        _pathfinding = new Pathfinding(null, null, null, "scoring");
        return null;
    }

    public ArrayList<Trajectory> redBottomScoring(){
        _pathfinding = new Pathfinding(null, null, null, "scoring");
        return null;
    }

    public ArrayList<Trajectory> redMidScoring(){
        _pathfinding = new Pathfinding(null, null, null, "scoring");
        return null;
    }

    public ArrayList<Trajectory> redTopScoring(){
        _pathfinding = new Pathfinding(null, null, null, "scoring");        
        return null;
    }

    public ArrayList<Trajectory> blueBottomDocking(){
        _pathfinding = new Pathfinding(null, null, null, "docking");
        return null;
    }

    public ArrayList<Trajectory> blueMidDocking(){
        _pathfinding = new Pathfinding(null, null, null, "docking");
        return null;
    }

    public ArrayList<Trajectory> blueTopDocking(){
        _pathfinding = new Pathfinding(null, null, null, "docking");
        return null;
    }

    public ArrayList<Trajectory> redBottomDocking(){
        _pathfinding = new Pathfinding(null, null, null, "docking");
        return null;
    }

    public ArrayList<Trajectory> redMidDocking(){
        _pathfinding = new Pathfinding(null, null, null, "docking");
        return null;
    }

    public ArrayList<Trajectory> redTopDocking(){
        _pathfinding = new Pathfinding(null, null, null, "docking");
        return null;
    }
}
