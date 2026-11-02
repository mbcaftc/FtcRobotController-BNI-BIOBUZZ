package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue.FarHiveTip;

@Autonomous(name = "AndrewAutoTest", group = "Debug")
public class AndrewPathingTest extends AndewFarHiveTip {
    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        //Set BlueMain points (BEFORE buildPaths())
        startPose = new AndrewPose(83, 132);
        nav1 = new AndrewPose(108, 108);
        nav2 = new AndrewPose (124, 72 );
        parkPose = new AndrewPose(128, 47);
        //BlueMain init
        buildPaths();
        //Start the follower
        follower.setPose(startPose.poseHeadingDegrees(270));
        follower.update();
    }
}
