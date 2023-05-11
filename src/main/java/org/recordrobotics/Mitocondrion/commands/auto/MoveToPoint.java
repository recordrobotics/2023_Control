package org.recordrobotics.Mitocondrion.commands.auto;

import edu.wpi.first.math.geometry.Transform3d;

public class MoveToPoint {

    private Transform3d _start;
    private Transform3d _end;
    
    public MoveToPoint (Transform3d start, Transform3d end){
        
        _start = start;
        _end = end;

    }

}
