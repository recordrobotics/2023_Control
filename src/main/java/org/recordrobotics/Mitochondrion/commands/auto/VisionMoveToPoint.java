package org.recordrobotics.Mitochondrion.commands.auto;

import org.photonvision.PhotonCamera;
import org.recordrobotics.Mitochondrion.subsystems.Drive;
import org.recordrobotics.Mitochondrion.subsystems.NavSensor;
import org.recordrobotics.Mitochondrion.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VisionMoveToPoint extends CommandBase {
    
    private Vision _vision;
    private Drive _drive;
    private double[] _currentPos;
    private double[] _targetPos;//Currently only dealing with X, as that is all that matters for chargestation
    private double[] _tolerance = {0.01, 0, 0};
    //private double[] _toMove;
    private double _speed;
    private PhotonCamera _cam;
    private double _side; // 0 = blue, 1 = red

public VisionMoveToPoint (Vision vision, Drive drive, double[] targetPos, PhotonCamera cam, double side){
    
    _vision = vision;
    _drive = drive;
    _targetPos = targetPos;
    _side = side;

    _speed = 0.2;

}

    @Override
    public void execute(){


        if (Vision.checkForTarget(_cam) == true) {


            _currentPos = Vision.estimateGlobalPose(_cam);

            if (_side == 0){
                _drive.move(-1*Math.signum(_targetPos[0] - _currentPos[0])*_speed, 0);
            }

            if (_side == 1){
                _drive.move(Math.signum(_targetPos[0] - _currentPos[0])*_speed, 0);
            }


        }

    }

    @Override
    public boolean isFinished(){
        return _currentPos[0] < _targetPos[0] + _tolerance[0] && _currentPos[0] > _targetPos[0] - _tolerance[0]; //TODO:
        
    }

}
