package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.AutoPaths;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;

//Same as Start_At_Hive, but starts one tile (24 inches) right of it and turns the turret 37.5 degrees clockwise while the launcher spins up
@Disabled
@Autonomous(name = "PROTOTYPE_Turret", group = "Drive")
public class FarTurretPrototype extends MovmentMain {
    //Degrees relative to the robot, negative is clockwise
    public double turretTarget = 37.5;

    public Pose startPose;
    public Pose nav1;
    public Pose parkPose;
    public Path start_to_park;

    public void buildPaths(){

        Pose p0 = new Pose(startPose.x(), startPose.y());
        Pose p1 = new Pose(nav1.x(), nav1.y());
        Pose p3 = new Pose(parkPose.x(), parkPose.y());


        start_to_park = path(
                line(p0, p1),
                line(p1, p3)
        ).constant(Math.toRadians(90));
    }

    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        //Set BlueMain points (BEFORE buildConstantPaths())
        startPose = new Pose(107, 8.8);
        nav1 = Quadrant3Back.pose();
        parkPose = ParkPoseB.pose();
        //BlueMain init
        this.buildPaths();
        //Start the follower
        follower.setPose(new Pose(startPose.x(), startPose.y(), Math.toRadians(90)));
        follower.update();
    }

    enum TurretAutoState {SHOOTING, DRIVING, CENTER_TURRET}
    TurretAutoState turretState = TurretAutoState.SHOOTING;

    //pathLogic runs every loop in LOOP, and again after a movePath with LOOP finishes

    @Override
    public void pathLogic(){
        switch (turretState) {
            case SHOOTING:

                    //Turn the turret while the launcher spins up
                    setTurretHeading(turretTarget);
                    launch(970, 980, 3, 1);
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
                if (!setTurretHeading(0)) {
                    turretServo.setPower(0);
                    //Hand off to MovmentMain's PARK state
                    pathState = pathingState.PARK;
                }
                break;
        }
    }
}
