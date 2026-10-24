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

    protected static final Pose pose1 = poseFactory.of(0, 0, 90);
    protected static final Pose pose2 = poseFactory.of(25, 0, 90);
    protected static final Pose pose3 = poseFactory.of(25, 70, 90);
    protected static final Pose pose4 = poseFactory.of(0, 70, 90);

    public Path one_two;
    public Path two_three;
    public Path three_four;
    public Path four_one;


    //When to go to park as failsafe (0-30 seconds from start, recommended 25)
    public double maxTimeBreakout = 25;

    //set up simple states
    public enum pathingState {START, MOVETOPOINT, RETURNMOVETOPOINT, PARK, END, LOOP}
    public pathingState pathState = pathingState.START;

    //State to return to after moveToPoint case
    public pathingState returnState;

    public Path moveToPointChain;


    public void pathGen(){
        one_two = create(pose1, pose2);
        two_three = create(pose2, pose3);
        three_four = create(pose3, pose4);
        four_one = create(pose4, pose1);
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
        follower.setPose(pose1);
        pathGen();
    }

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
                movePath(one_two, pathingState.LOOP);
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
                if (moveToPointChain.equals(one_two)) {
                    movePath(two_three, pathingState.LOOP);
                } else if (moveToPointChain.equals(two_three)) {
                    movePath(three_four, pathingState.LOOP);
                } else if (moveToPointChain.equals(three_four)) {
                    movePath(four_one, pathingState.LOOP);
                } else if (moveToPointChain.equals(four_one)) {
                    movePath(one_two, pathingState.LOOP);
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
        telemetry.update();

    }


}
