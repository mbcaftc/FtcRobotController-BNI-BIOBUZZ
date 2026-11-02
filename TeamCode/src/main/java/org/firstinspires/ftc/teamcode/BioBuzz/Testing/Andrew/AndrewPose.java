package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import com.pedropathing.math.Pose;

public class AndrewPose {
    public double x;
    public double y;
    public AndrewPose(double xPos, double yPos){
        this.x = xPos;
        this.y = yPos;
    }
    public Pose pose(){
        return new Pose(this.x, this.y, 0);
    }
    public Pose poseHeadingDegrees(double deg){
        return new Pose(this.x, this.y, Math.toRadians(deg));
    }
}
