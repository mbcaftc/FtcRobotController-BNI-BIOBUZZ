package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Comp_3;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.AutoTemplates.FarHiveTip;

//Same as Start_At_Hive, but starts one tile (24 inches) right of it and turns the turret 37.5 degrees clockwise while the launcher spins up
@Autonomous(name = "Comp3_Start_At_Hive_Right", group = "Comp3")
public class BlueStartFar2NavTurret extends FarHiveTip {
    //Degrees relative to the robot, negative is clockwise
    public double turretTarget = -36;

    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        //Set BlueMain points (BEFORE buildConstantPaths())
        startPose = StartFarLeftTileRight.pose();
        nav1 = Quadrant1Center.pose();
        nav2 = MidAroundCloseHumans.pose();
        parkPose = ParkPoseA.pose();
        //BlueMain init
        buildPaths();
        //Start the follower
        follower.setPose(new Pose(startPose.x(), startPose.y(), Math.toRadians(270)));
        follower.update();
    }

    enum TurretAutoState {START, SHOOTING, DRIVING, CENTER_TURRET}
    TurretAutoState turretState = TurretAutoState.START;

    //pathLogic runs every loop in LOOP, and again after a movePath with LOOP finishes
    @Override
    public void pathLogic(){
        switch (turretState) {
            case START:
                primeForLaunch();
                turretState = TurretAutoState.SHOOTING;
                break;
            case SHOOTING:
                //Turn the turret while the launcher spins up
                turretAngle = turretTarget;
                burnerLaunch(990, 2);
                if (opmodeTimer.seconds() > 10) {
                    cutPower();
                    //CRServo keeps its last power, so stop it before driving
                    turretServo.setPower(0);
                    turretState = TurretAutoState.DRIVING;
                    //Come back to pathLogic (LOOP) after parking instead of going straight to PARK
                    movePath(start_to_park, pathingState.LOOP);
                }
                break;
            case DRIVING:
                //Only runs once the park path is done
                turretState = TurretAutoState.CENTER_TURRET;
                break;
            case CENTER_TURRET:
                //Called every loop so it stops at 0 (returns false once inside the target window)
                turretAngle = 0;
                break;
        }
    }
}
