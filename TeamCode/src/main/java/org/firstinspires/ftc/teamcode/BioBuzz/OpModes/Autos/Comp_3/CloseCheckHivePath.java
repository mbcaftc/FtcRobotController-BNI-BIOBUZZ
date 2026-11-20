package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Comp_3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew.IntakeAutos.CloseCheckHive;

@Autonomous(name = "Comp3_CheckHivesClose", group = "Comp3")
public class CloseCheckHivePath extends CloseCheckHive {
    @Override
    public void init(){
        //Starting Position
        startPose = StartCloseLeft.setDeg(90);

        shootingPose = Quadrant3Back.setDeg(0);

        parkingPose = ParkPoseB.setDeg(180);

        shotTime = 10;

        //Shot Angles
        shotAngle = 0;

        //Velocities
        shotVelocity = 965;

        initialDelay = 13;

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
