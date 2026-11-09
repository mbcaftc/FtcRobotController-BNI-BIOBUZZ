package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ShootFarUnderHive", group = "Debug")
public class ShootFarUnderHive extends ShootFarUnderHiveBase {
    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        //Set points (BEFORE buildPaths())
        startPose = StartFarLeft.setDeg(270);
        nav1 = MidUnderHive.setDeg(270);
        nav2 = Quadrant3Center.setDeg(270);
        parkPose = ParkPoseB.setDeg(270);
        //Build paths
        buildPaths();
        //Start the follower
        follower.setPose(startPose);
        follower.update();
    }
}
