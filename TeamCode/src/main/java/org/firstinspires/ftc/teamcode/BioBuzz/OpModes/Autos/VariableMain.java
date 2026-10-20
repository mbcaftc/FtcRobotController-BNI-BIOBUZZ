package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.AtomicPath;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

public class VariableMain extends AutoMain{


    protected static final PoseFactory poseFactory = PoseFactory.degrees();

    protected static final Pose start = poseFactory.of(0, 0, 90);
    protected static final Pose path1 = poseFactory.of(40, 40, 0);

    public Path start_to_fire;

    //When to go to park as failsafe (0-30 seconds from start, recommended 25)
    public double maxTimeBreakout = 25;

    //set up simple states
    public enum pathingState {START, MOVETOPOINT, RETURNMOVETOPOINT, PARK, END}
    public pathingState pathState = pathingState.START;

    //State to return to after moveToPoint case
    public pathingState returnState;

    public Path moveToPointChain;


    public void pathGen(){
        start_to_fire = create(start, path1);
    }

    private void movePath(Path path, pathingState state){
        moveToPointChain = path;
        returnState = state;
        pathState = pathingState.MOVETOPOINT;
    }

    @Override
    public void start(){
        opmodeTimer.reset();
        pathTimer.reset();
        pathState = pathingState.START;
        follower.setPose(start);
        pathGen();
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Current time :", " %.2f", opmodeTimer.milliseconds() / 1000.0);
        telemetry.addData("Current state :", pathState);
        telemetry.update();

        //Max time breakout
        if(opmodeTimer.seconds() > maxTimeBreakout){
            if(pathState != pathingState.END){
                telemetry.addLine("EMERGENCY BREAKOUT. PARKING.");
                pathState = pathingState.PARK;
            }
        }

        switch (pathState) {
            case START:
                //Move to the first spike
                movePath(start_to_fire, pathingState.PARK);
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

            case PARK:
                if (!(follower.isBusy())) {
                    pathState = pathingState.END;
                }
                break;
            case END:
                break;
        }
    }


}
