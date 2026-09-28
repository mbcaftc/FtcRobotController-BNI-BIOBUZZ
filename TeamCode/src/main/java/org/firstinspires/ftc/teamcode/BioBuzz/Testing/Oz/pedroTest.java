package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Oz;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.BioBuzz.pedro.Constants;

import java.util.List;

@Autonomous(name = "Pedro test oz v1", group = "Drive")
public class pedroTest extends OpMode {

    private Follower follower;
    private List<Path> paths;
    private int pathState;
    private boolean finished;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(27.2335, 36.7775, 180);
    private final Pose point2 = poseFactory.of(25.625, 77.614, -87.7443);
    private final Pose point3 = poseFactory.of(26.9547, 113.158, -92.1424);


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        paths = EasyCreate.create(start, path1, point2, point3);
    }

    @Override
    public void init_loop() {
        ;
    }

    @Override
    public void start() {
        pathState = 0;
        finished = false;
    }

    @Override
    public void loop() {
        if (finished) {
            telemetry.addData("Status", "Complete");
            return;
        }

        switch (pathState) {
            case 0:
                follower.follow(paths.get(0));
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.follow(paths.get(1));
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.follow(paths.get(2));
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    finished = true;
                    follower.stop();
                }
                break;
        }

        if (!finished) {
            follower.update();
        }
        telemetry.addData("Path", "%d / %d", pathState, paths.size());
        telemetry.addData("Pose", follower.pose());
        telemetry.addData("Status", finished ? "Complete" : "Following");
    }

    @Override
    public void stop() {
        follower.stop();
    }
}
