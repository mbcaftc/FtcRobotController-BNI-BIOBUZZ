package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew.IntakeAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CheckHivesClose", group = "Close")
public class CloseCheckHivePath extends CloseCheckHive {
    @Override
    public void init(){
        //Starting Position
        startPose = StartCloseLeft.setDeg(90);

        shootingPose = Quadrant3Back.setDeg(0);

        parkingPose = ParkPoseB.setDeg(180);

        shotTime = 7.5;

        //Shot Angles
        shotAngle = 0;

        //Velocities
        shotVelocity = 955;

        initialDelay = 17;

        //AutoMain init
        setup();
        initHardWare();
        //Set the breakout time to 29s because auto is long
        maxTimeBreakout = 27;
        //Set up the paths
        buildPaths();
        //Start the follower
        follower.setPose(startPose);
        follower.update();
    }

}
