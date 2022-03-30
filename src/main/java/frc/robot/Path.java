package frc.robot;

import java.util.List;

import javax.swing.text.Position;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Path {
    public Pose2d start;
    public List<Translation2d> waypoints;
    public Pose2d end;

    public Path(){

    }
    public Path(Pose2d start, List<Translation2d> waypoints, Pose2d end){
        this.start = start;
        this.waypoints = waypoints;
        this.end = end;
    }
    public Pose2d getStart(){
        return start;
    }
    public Pose2d getEnd(){
        return end;
    }
    public List<Translation2d> getWaypoints(){
        return waypoints;
    }

    public Pose2d setStart(){
        return start;
    }
    public Pose2d setEnd(){
        return end;
    }
    public List<Translation2d> setWaypoints(){
        return waypoints;
    }
    
}
