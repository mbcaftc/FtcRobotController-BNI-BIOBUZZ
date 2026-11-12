package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew.UniPose;

public class MovmentMain extends AutoMain{
    protected static final PoseFactory poseFactory = PoseFactory.degrees();

    protected static UniPose StartCloseLeft = new UniPose(83, 8.8);//1
    protected static  UniPose StartCloseRight = new UniPose(106, 8.8);//2
    protected static  UniPose MidUnderHive = new UniPose(84, 72);//3

    protected static  UniPose MidAroundCloseHive = new UniPose(84, 72);//this is close to flower//4
    protected static  UniPose MidAroundCloseHumans = new UniPose (124, 72 );//this is close to humans//5
    protected static  UniPose StartFarLeft = new UniPose(83, 132);//6
    protected static final UniPose StartFarLeftTileRight = new UniPose(107, 132);//StartFarLeft moved one tile (24 in) right
    protected static final UniPose StartRightRight = new UniPose(106, 132);//7

    protected static final UniPose ParkPoseA = new UniPose(128, 47);//11
    protected static final UniPose ParkPoseB = new UniPose(128, 25);//12
    protected static final UniPose LooseBallsPickup =  new UniPose(133, 130);//13
    protected static final UniPose PreLooseBallsPickup =  new UniPose(110, 130);//
    protected static final UniPose Quadrant1Center = new UniPose(108, 108);//15
    protected static final UniPose Quadrant2Center = new UniPose(72, 108);//14
    protected static final UniPose Quadrant3Center = new UniPose(83, 36);//16
    protected static final UniPose Quadrant3Back = new UniPose(83, 16);//
    protected static final UniPose Quadrant4Center = new UniPose(108, 36);//17
    protected static final UniPose FlowerClosePos = new UniPose(95, 8.8);//18
    protected static final UniPose FlowerCloseAround = new UniPose(95, 18);//19
    protected static final UniPose FlowerFarPos = new UniPose(132, 94);//22
    protected static final UniPose FlowerFarAround = new UniPose(122, 94);//22
    protected static final UniPose FlowerCloseWest= new UniPose(80, 8.8);//18
    protected static final UniPose FlowerCloseEast = new UniPose(115, 8.8 );//19
    protected static final UniPose FlowerFarNorth = new UniPose(132, 94);//22
    protected static final UniPose FlowerFarSouth = new UniPose(122, 94);//22


    //When to go to park as failsafe (0-30 seconds from start, recommended 25)
    public double maxTimeBreakout = 25;
    //Where the auto parks, set it in init() so the failsafe can drive here
    public Pose parkPose;
    public boolean failsafeTriggered = false;

    //set up simple states
    public enum pathingState {START, MOVETOPOINT, RETURNMOVETOPOINT, PARK, END, LOOP}
    public pathingState pathState = pathingState.START;

    //State to return to after moveToPoint case
    public pathingState returnState;
    public Path moveToPointChain;
    public void movePath(Path path, MovmentMain.pathingState state){
        moveToPointChain = path;
        returnState = state;
        pathState = pathingState.MOVETOPOINT;
    }
    public void pathLogic(){}//this is being overwritten
    @Override
    public void loop() {
        follower.update();
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("R", follower.pose().heading());
        telemetry.addData("Current time :", " %.2f", opmodeTimer.milliseconds() / 1000.0);
        telemetry.addData("Current state :", pathState);
        telemetry.addData("Pollen RPM: ", launcherMotor.getVelocity());

        //Max time breakout, runs once and drives straight to parkPose unless the auto is already parking
        if(opmodeTimer.seconds() > maxTimeBreakout && !failsafeTriggered){
            failsafeTriggered = true;
            //pathLogic stops running after this, so don't leave the CR turret spinning at its last power
            turretServo.setPower(0);
            boolean alreadyParking = pathState == pathingState.PARK || pathState == pathingState.END
                    || ((pathState == pathingState.MOVETOPOINT || pathState == pathingState.RETURNMOVETOPOINT)
                        && returnState == pathingState.PARK);
            if(!alreadyParking){
                cutPower();
                if(parkPose != null){
                    Pose here = follower.pose();
                    //Hold the current heading (no .linear) so it doesn't turn near walls on the way
                    Path failsafePark = path(line(new Pose(here.x(), here.y()), new Pose(parkPose.x(), parkPose.y())))
                            .constant(here.heading());
                    movePath(failsafePark, pathingState.PARK);
                }
                else{
                    pathState = pathingState.PARK;
                }
            }
        }
        if(failsafeTriggered){
            telemetry.addLine("FAILSAFE: PARKING.");
        }

        switch (pathState) {
            case START:
                pathState = pathingState.LOOP;
                moveToPointChain = createLinearPath(new Pose(0, 0), new Pose(1, 0), 0, 0);
                break;

            case MOVETOPOINT:
                follower.follow(moveToPointChain);
                pathTimer.reset(); // Reset timer to give the follower time to initialize
                pathState = pathingState.RETURNMOVETOPOINT;
                break;

            case RETURNMOVETOPOINT:
                // Wait at least 250ms for the follower to pick up the path and become busy
                if (pathTimer.milliseconds() > 250 && !follower.isBusy()) {
                    pathState = returnState;
                }
                break;


            case LOOP:
                pathLogic();
                break;


            case PARK:
                if (!(follower.isBusy())) {
                    pathState = pathingState.END;
                }
                break;

            case END:
                break;
        }

        telemetry.update();
    }
}
