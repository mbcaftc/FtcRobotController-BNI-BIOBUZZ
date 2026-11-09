package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AndrewTwoHive", group = "Debug")
public class AndrewPathingTwoHive extends AndewFarDoubleHive {
    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        //Set BlueMain points (BEFORE buildConstantPaths())
        startPose = StartFarLeft.setDeg(270);
        nav1 = MidUnderHive.setDeg(270);
        nav2 = Quadrant3Center.setDeg(270);
        shootingPose = new UniPose(83, 22).setDeg(90); //Quadrant3Back (y=16) left too little room before the audience wall
        nav3 = Quadrant4Center.setDeg(135);
        parkPose = ParkPoseB.setDeg(180);
        //BlueMain init
        buildPaths();
        //Start the follower
        follower.setPose(startPose);
        follower.update();
    }
}
