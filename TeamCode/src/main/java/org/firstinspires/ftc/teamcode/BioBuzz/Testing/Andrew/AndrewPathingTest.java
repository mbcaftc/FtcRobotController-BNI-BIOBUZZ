package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AndrewAutoTest", group = "Debug")
public class AndrewPathingTest extends AndewFarHiveTip {
    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        //Set BlueMain points (BEFORE buildPaths())
        startPose = new UniPose(StartFarLeft).setDeg(270);
        nav1 = new UniPose(Quadrant1Center).pose();
        nav2 = new UniPose(MidAroundCloseHumans).pose();
        parkPose = new UniPose(ParkPoseA).pose();
        //BlueMain init
        buildPaths();
        //Start the follower
        follower.setPose(startPose);
        follower.update();
    }
}
