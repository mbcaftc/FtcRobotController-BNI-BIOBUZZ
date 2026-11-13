package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Autos.Testing;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;
import org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew.UniPose;
import org.firstinspires.ftc.teamcode.BioBuzz.pedro.MainConstants;

//Start facing 270, shoot, back off the wall, turn sideways, slowly intake into the corner, hold it, back out, park
//Only uses constant headings (no .linear) so the robot never turns close to a wall
@Disabled
@Autonomous(name = "Start_Far_PickUp_oz", group = "Drive")
public class BlueStartFar4PickUp extends MovmentMain {

    protected static final UniPose OffWall = new UniPose(86, 114);
    //Short sideways move where the robot turns from 270 to 0
    protected static final UniPose TurnPoint = new UniPose(95, 114);

    public double startHeading = Math.toRadians(270);
    public double sideHeading = Math.toRadians(0);//Intake side faces the loose balls
    public double intakeSpeed = 0.3;//Fraction of max path speed while intaking
    public double cornerHoldTime = 2;//Seconds to hold the corner with the intake running
    public double shootSpeed = 1030;//Launcher velocity for the opening shot
    public double shootEndTime = 9;//Seconds after start when the opening shot stops and the robot leaves the wall

    enum ScoringState {SHOOTING, OFF_WALL, TURN, TO_PICKUP, INTO_CORNER, HOLD_CORNER, BACK_OUT, PARK}
    ScoringState currentState = ScoringState.SHOOTING;

    //Protected so autos that extend this one (FarCheckHive) can reuse the same paths
    protected Path offWall;
    protected Path turnSideways;
    protected Path toPickup;
    protected Path intoCorner;
    protected Path backOut;
    protected Pose cornerPose;
    protected Timer holdTimer = new Timer();
    protected Path park;

    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();

        Pose start = StartFarLeft.pose();
        Pose offWallPose = OffWall.pose();
        Pose turnPose = TurnPoint.pose();
        Pose prePickup = PreLooseBallsPickup.pose();
        Pose loose = LooseBallsPickup.pose();
        Pose q1 = Quadrant1Center.pose();
        parkPose = ParkPoseA.pose();//Also used by the failsafe park

        //Straight off the wall, still facing 270
        offWall = path(line(start, offWallPose)).constant(startHeading);
        //Short slide while turning to 0, away from the wall
        turnSideways = path(line(offWallPose, turnPose)).constant(sideHeading);
        //Line up in front of the loose balls
        toPickup = path(line(turnPose, prePickup)).constant(sideHeading);
        //Drive slowly into the loose balls and back out with the intake on
        //maxPathSpeed.at() caps the speed for just this path
        cornerPose = new Pose(loose.x(), loose.y(), sideHeading);
        intoCorner = path(line(prePickup, loose)).constant(sideHeading)
                .with(MainConstants.foresightConfig.maxPathSpeed.at(intakeSpeed));
        backOut = path(line(loose, prePickup)).constant(sideHeading)
                .with(MainConstants.foresightConfig.maxPathSpeed.at(intakeSpeed));
        //Park, keep facing sideways so it never turns
        park = path(line(prePickup, q1), line(q1, parkPose)).constant(sideHeading);

        //Start the follower
        follower.setPose(new Pose(start.x(), start.y(), startHeading));
        follower.update();
    }

    @Override
    public void start(){
        super.start();//Reset timers so time spent in init doesn't count
        currentState = ScoringState.SHOOTING;
    }

    //pathLogic only runs in the LOOP state, so each case starts after the previous path has finished
    @Override
    public void pathLogic(){
        switch(currentState){
            case SHOOTING:
                launch(shootSpeed, 980, 3, 1);
                if (opmodeTimer.seconds() > shootEndTime) {
                    cutPower();
                    currentState = ScoringState.OFF_WALL;
                    movePath(offWall, pathingState.LOOP);
                }
                break;
            case OFF_WALL:
                currentState = ScoringState.TURN;
                movePath(turnSideways, pathingState.LOOP);
                break;
            case TURN:
                currentState = ScoringState.TO_PICKUP;
                movePath(toPickup, pathingState.LOOP);
                break;
            case TO_PICKUP:
                //Intake stays on from here until the robot has backed out of the corner
                intake(-1);
                currentState = ScoringState.INTO_CORNER;
                movePath(intoCorner, pathingState.LOOP);
                break;
            case INTO_CORNER:
                //In the corner, hold the pose while the intake runs
                follower.hold(cornerPose);
                holdTimer.reset();
                currentState = ScoringState.HOLD_CORNER;
                break;
            case HOLD_CORNER:
                if (holdTimer.seconds() > cornerHoldTime) {
                    currentState = ScoringState.BACK_OUT;
                    movePath(backOut, pathingState.LOOP);
                }
                break;
            case BACK_OUT:
                intake(0);
                currentState = ScoringState.PARK;
                movePath(park, pathingState.PARK);
                break;
            case PARK:
                break;
        }
    }
}
