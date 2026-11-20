package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew.IntakeAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew.UniPose;

@Autonomous(name = "IntakeAutoCheckCenterBlueNoTurn", group = "Intake")
public class IntakeCheckTagsCenterNoTurn extends IntakeCheckHiveAuto {
    @Override
    public void init(){
        //Starting Position
        startPose = StartFarLeft.setDeg(0);
        //Navigate to the intake position
        nav1 = TurnPoint.setDeg(0);
        //Start intake position
        startIntake = PreLooseBallsPickup.setDeg(0);
        //End of intake position
        endIntake = LooseBallsPickup.setDeg(0);
        //Control point for returning to shooting pose
        nav2 = TurnPoint.setDeg(0);
        //Second shooting pose
        secondShotFar = StartFarLeft.setDeg(0);
        //Control points for parking navigation
        nav3Far = MidUnderHive.setDeg(0);
        nav4Far = Quadrant3Center.setDeg(0);

        nav3Close = Quadrant3Center.setDeg(0);
        secondShotClose = Quadrant3Back.setDeg(270);
        hiveNavPoint = new UniPose(83, 18).setDeg(270);


        //Parking Pose
        parkingPose = ParkPoseB.setDeg(180);

        blueSide = true;


        //How long each shot may run
        firstShotTime = 7.5;
        secondShotTime = 7.5;

        //Shot Angles
        firstShotAngle = -90;
        secondShotAngle = -180;

        //Velocities
        firstShotVelocity = 955;
        secondShotVelocity = 955;

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
