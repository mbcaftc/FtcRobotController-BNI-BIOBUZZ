package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class MovmentMain extends AutoMain{


    protected static final PoseFactory poseFactory = PoseFactory.degrees();

    protected static  Pose StartCloseLeft = new Pose(83, 8.8);//1
    protected static  Pose StartCloseRight = new Pose(106, 8.8);//2
    protected static  Pose MidUnderHive = new Pose(84, 72);//3

    protected static  Pose MidAroundCloseHive = new Pose(84, 72);//this is close to flower//4
    protected static  Pose MidAroundCloseHumans = new Pose (124, 72 );//this is close to humans//5
    protected static  Pose StartFarLeft = new Pose(83, 132);//6
    protected static final Pose StartRightRight = new Pose(106, 132);//7

    protected static final Pose ParkPoseA = new Pose(124, 72);//11
    protected static final Pose ParkPoseB = new Pose(124, 72);//12
    protected static final Pose LooseBallsPickup =  new Pose(132, 133);//13
    protected static final Pose Quadrant1Center = new Pose(108, 108);//15
    protected static final Pose Quadrant3Center = new Pose(72, 36);//16
    protected static final Pose Quadrant4Center = new Pose(108, 36);//17
    protected static final Pose FlowerClosePos = new Pose(95, 8.8);//18
    protected static final Pose FlowerCloseAround = new Pose(95, 18);//19
    protected static final Pose FlowerFarPos = new Pose(132, 94);//22
    protected static final Pose FlowerFarAround = new Pose(122, 94);//22
    protected static final Pose FlowerCloseWest= new Pose(80, 8.8);//18
    protected static final Pose FlowerCloseEast = new Pose(115, 8.8 );//19
    protected static final Pose FlowerFarNorth = new Pose(132, 94);//22
    protected static final Pose FlowerFarSouth = new Pose(122, 94);//22





    //When to go to park as failsafe (0-30 seconds from start, recommended 25)
    public double maxTimeBreakout = 25;

    //set up simple states
    public enum pathingState {START, MOVETOPOINT, RETURNMOVETOPOINT, PARK, END, LOOP}
    public pathingState pathState = pathingState.START;

    //State to return to after moveToPoint case
    public pathingState returnState;

    public Path moveToPointChain;



    private void movePath(Path path, MovmentMain.pathingState state){
        moveToPointChain = path;
        returnState = state;
        pathState = pathingState.MOVETOPOINT;
    }

    @Override
    public void start(){
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

        //Max time breakout
        if(opmodeTimer.seconds() > maxTimeBreakout){
            if(pathState != pathingState.END){
                telemetry.addLine("EMERGENCY BREAKOUT. PARKING.");
                pathState = pathingState.PARK;
            }
        }

        switch (pathState) {
            case START:
                //follower.manual(0, 0, 0.3);
                //movePath(one_two, pathingState.LOOP);
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
