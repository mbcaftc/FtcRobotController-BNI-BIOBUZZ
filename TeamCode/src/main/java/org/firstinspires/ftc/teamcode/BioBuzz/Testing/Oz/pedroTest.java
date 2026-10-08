package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Oz;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.pedro.ProgramConstants;

import java.util.EnumMap;
import java.util.Map;


@Autonomous(name = "Pedro test oz v1", group = "Drive")
public class pedroTest extends PoseMain {

    private Follower follower;
    private final Map<PathState, Path> paths = new EnumMap<>(PathState.class);
    private boolean pathStarted;

    private PathState pathState;
    private boolean finished;
    private Timer runTimer;
    private static final double OverTimeCancelTime = 25.0;
    private static final double UltraLowTime = 29.0;




    @Override
    public void init() {
        runTimer = new Timer();
        follower = ProgramConstants.create(hardwareMap);
        follower.setPose(start);
        paths.clear();
        for (Object[] definition : PathState.pathDefinitions()) {
            PathState name = PathState.valueOf((String) definition[0]);
            Pose startPose = (Pose) definition[1];
            Pose endPose = (Pose) definition[2];
            paths.put(name, FunctionMain.create(startPose, endPose));
        }

    }


    @Override
    public void start() {
        runTimer.reset();
        pathState = PathState.FIRST_PATH;
        pathStarted = false;
        finished = false;
    }

    @Override
    public void loop() {

        if (follower.currentPath() != null) {
            telemetry.addData("path distance remaining", follower.distanceToEndpoint());
            telemetry.addData("Path", follower.currentPath());
        }


        double elapsedSeconds = runTimer.seconds();
        if (!finished) {
            if (elapsedSeconds >= UltraLowTime) {
                pathState = PathState.ULTRA_LOW_TIME;
            } else if (elapsedSeconds >= OverTimeCancelTime
                    && pathState != PathState.LOW_TIME
                    && pathState != PathState.WAIT_FOR_RETURN_PATH) {
                pathState = PathState.LOW_TIME;
            }
        }

        switch (pathState) {
            case FIRST_PATH:
                followPathThen(PathState.SECOND_PATH);
                break;
            case SECOND_PATH:
                followPathThen(PathState.THIRD_PATH);
                break;
            case THIRD_PATH:
                followPathThen(PathState.FINISHED);
                break;
            case FINISHED:
                follower.stop();
                finished = true;
                break;
            case LOW_TIME:
                follower.stop();
                follower.follow(FunctionMain.create(follower.pose(), start));
                pathState = PathState.WAIT_FOR_RETURN_PATH;
                break;
            case WAIT_FOR_RETURN_PATH:
                if (!follower.isBusy()) {
                    follower.stop();
                    finished = true;
                    pathState = PathState.FINISHED;
                }
                break;
            case ULTRA_LOW_TIME:
                follower.stop();
                finished = true;
                break;
            default:
                break;
        }

        if (!finished) {
            follower.update();
        }
        telemetry.addData("Path state", pathState);
        telemetry.addData("Pose", follower.pose());
        telemetry.addData("Status", finished);
    }

    private void followPathThen(PathState nextState) {
        if (!pathStarted) {
            follower.follow(paths.get(pathState));
            pathStarted = true;
        } else if (!follower.isBusy()) {
            pathState = nextState;
            pathStarted = false;
            if (nextState == PathState.FINISHED) {
                follower.stop();
                finished = true;
            }
        }
    }

    @Override
    public void stop() {
        follower.stop();//pedro recomends this
        telemetry.addLine("Stopped by controller");
        telemetry.addData("last Path", follower.currentPath());
    }
}
