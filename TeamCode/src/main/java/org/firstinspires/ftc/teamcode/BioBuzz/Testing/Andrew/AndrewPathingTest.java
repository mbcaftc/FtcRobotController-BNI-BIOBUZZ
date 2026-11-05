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
        startPose = StartFarLeft.setDeg(270);
        nav1 = Quadrant1Center.setDeg(270);
        nav2 = MidAroundCloseHumans.setDeg(270);
        parkPose = ParkPoseA.setDeg(180);
        //BlueMain init
        buildPaths();
        //Start the follower
        follower.setPose(startPose);
        follower.update();
    }
}
