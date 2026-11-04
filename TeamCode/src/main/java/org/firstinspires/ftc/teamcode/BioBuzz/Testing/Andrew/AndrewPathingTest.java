package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AndrewAutoTest", group = "Debug")
public class AndrewPathingTest extends AndewFarHiveTip {
    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        //Set BlueMain points (BEFORE buildConstantPaths())
        startPose = new UniPose(StartFarLeft).setDeg(270);
        nav1 = new UniPose(Quadrant1Center).setDeg(270);
        nav2 = new UniPose(MidAroundCloseHumans).setDeg(270);
        parkPose = new UniPose(ParkPoseA).setDeg(180);
        //BlueMain init
        buildPaths();
        //Start the follower
        follower.setPose(startPose);
        follower.update();
    }
}
