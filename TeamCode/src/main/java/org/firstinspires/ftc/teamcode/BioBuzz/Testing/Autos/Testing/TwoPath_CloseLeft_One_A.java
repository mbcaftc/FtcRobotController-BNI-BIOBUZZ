package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Autos.Testing;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.AutoTemplates.TwoPathMain;
@Disabled
@Autonomous(name = "Blue_NearCenter_One_A", group = "Drive")
public class TwoPath_CloseLeft_One_A extends TwoPathMain {
    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        //Set BlueMain points (BEFORE buildConstantPaths())
        startPose = StartCloseLeft.pose();
        shootPose = Quadrant3Back.pose();
        parkPose = ParkPoseB.pose();
        //BlueMain init
        buildPaths();
        //Start the follower
        follower.setPose(new Pose(startPose.x(), startPose.y(), Math.toRadians(90)));
        follower.update();
    }
}
