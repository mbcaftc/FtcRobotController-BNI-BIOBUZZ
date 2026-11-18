package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew.IntakeAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "IntakeAutoCheckCenterBlue", group = "Intake")
public class IntakeCheckTagsCenter extends IntakeCheckHiveAuto {
    @Override
    public void init(){
        //Starting Position
        startPose = StartFarLeft.setDeg(270);
        //Navigate to the intake position
        nav1 = TurnPoint.setDeg(0);
        //Start intake position
        startIntake = PreLooseBallsPickup.setDeg(0);
        //End of intake position
        endIntake = LooseBallsPickup.setDeg(0);
        //Control point for returning to shooting pose
        nav2 = TurnPoint.setDeg(180);
        //Second shooting pose
        secondShotFar = StartFarLeft.setDeg(270);
        //Control points for parking navigation
        nav3Far = MidUnderHive.setDeg(270);
        nav4Far = Quadrant3Center.setDeg(270);

        nav3Close = Quadrant3Center.setDeg(270);

        //Parking Pose
        parkingPose = ParkPoseB.setDeg(180);

        blueSide = true;


        //How long each shot may run
        firstShotTime = 7.5;
        secondShotTime = 7.5;

        //Shot Angles
        firstShotAngle = 0;
        secondShotAngle = 0;

        //Velocities
        firstShotVelocity = 1080;
        secondShotVelocity = 1080;

        //AutoMain init
        setup();
        initHardWare();
        //Set the breakout time to 29s because auto is long
        maxTimeBreakout = 29;
        //Set up the paths
        buildPaths();
        //Start the follower
        follower.setPose(startPose);
        follower.update();
    }

}
