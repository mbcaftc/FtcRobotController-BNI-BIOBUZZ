package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Start_At_Hive", group = "Drive")
public class BlueStartFar2Nav extends FarHiveTip{
    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        //Set BlueMain points (BEFORE buildPaths())
        startPose = StartFarLeft;
        nav1 = Quadrant1Center;
        nav2 = MidAroundCloseHumans;
        parkPose = ParkPoseB;
        //BlueMain init
        buildPaths();
        //Start the follower
        follower.setPose(new Pose(startPose.x(), startPose.y(), Math.toRadians(270)));
        follower.update();
    }
}
