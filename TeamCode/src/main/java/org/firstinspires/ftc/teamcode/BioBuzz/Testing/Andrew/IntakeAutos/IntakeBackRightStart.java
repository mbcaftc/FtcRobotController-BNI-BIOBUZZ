package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew.IntakeAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "IntakeAutoBackCorner", group = "Intake")
public class IntakeBackRightStart extends IntakeAuto {
    @Override
    public void init(){
        //Starting Position
        startPose = RightBackCorner.setDeg(90);
        //Navigate to the intake position
        nav1 = TurnPoint.setDeg(0);
        //Start intake position
        startIntake = PreLooseBallsPickup.setDeg(0);
        //End of intake position
        endIntake = LooseBallsPickup.setDeg(0);
        //Control point for returning to shooting pose
        nav2 = TurnPoint.setDeg(180);
        //Second shooting pose
        secondShot = StartFarLeft.setDeg(270);
        //Control points for parking navigation
        nav3 = MidUnderHive.setDeg(270);
        nav4 = Quadrant3Center.setDeg(270);
        //Parking Pose
        parkingPose = ParkPoseB.setDeg(180);

        //How long each shot may run
        firstShotTime = 7.5;
        secondShotTime = 7.5;

        //Shot Angles
        firstShotAngle = 49.5;
        secondShotAngle = 0;

        //Velocities
        firstShotVelocity = 970;
        secondShotVelocity = 950;

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
