package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import com.pedropathing.math.Pose;

public class UniPose {
    public double x;
    public double y;
    public UniPose(Pose thisPose){
        this.x = thisPose.x();
        this.y = thisPose.y();
    }
    public UniPose(double xPos, double yPos){
        this.x = xPos;
        this.y = yPos;
    }
    public Pose pose(){
        return new Pose(this.x, this.y, 0);
    }
    public Pose setDeg(double deg){
        return new Pose(this.x, this.y, Math.toRadians(deg));
    }
}
