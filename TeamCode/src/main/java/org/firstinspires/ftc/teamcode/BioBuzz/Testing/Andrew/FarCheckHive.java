package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addLimelight;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.Testing.Autos.Testing.BlueStartFar4PickUp;

import java.util.Collections;
import java.util.List;

//Runs the 4 pickup auto (shoot from StartFarLeft, intake the corner pollen), then drives back to the start pose and checks for the blue hive
//  - Tags seen: shoot from the start pose, then follow the ShootFarUnderHive path to park
//  - No tags: drive under the hive, turn around in place to face it (90), shoot, then park
//Reuses BlueStartFar4PickUp's paths and settings, so tuning that auto also tunes this one
@Autonomous(name = "Far_Check_Hive", group = "Drive")
public class FarCheckHive extends BlueStartFar4PickUp {
    //Blue hive April Tag IDs (same as WaitForTipBlue / BlueTeleOP)
    public int frontMinId = 38, frontMaxId = 41;
    public int backMinId = 42, backMaxId = 45;

    //Where the robot stops past the hive and turns around. x of 85 or more keeps the corners off the centerline (x 72) while turning. TUNE ON THE FIELD
    protected static final UniPose UnderShotPoint = new UniPose(86, 24);

    public double turnedHeading = Math.toRadians(90);//Facing back toward the hive after driving under it

    public double checkTime = 1;//Seconds at the start pose to look for the blue hive before driving under
    public double confirmSeconds = 0.5;//Tags have to be seen this long in a row (filters out flicker)
    public double secondShootSpeed = 960;//Launcher velocity for the second shot (same as Start_At_Hive / FarHiveTip)
    public double secondShootDuration = 5;//Seconds to shoot the corner pollen for
    public double turnTolerance = Math.toRadians(5);//Close enough to the turned heading to start shooting
    public double maxTurnTime = 2.5;//Shoot anyway if the turn hasn't settled by then
    public double intakePathTimeout = 1;//Seconds before the slow drive into the corner gives up and starts the corner hold
    //Failsafe park time for this auto only (MovmentMain default is 25). Late so the second shot isn't cut off,
    //and pushed back again once the second shot starts so it always gets its full secondShootDuration
    public double failsafeTime = 29;

    enum CheckState {SHOOTING, OFF_WALL, TURN_SIDEWAYS, TO_PICKUP, INTO_CORNER, HOLD_CORNER, BACK_OUT,
        RETURN_TURN_POINT, RETURN_OFF_WALL, RETURN_START, CHECK, SHOOT_START, DRIVE_UNDER, TURN_AROUND, SHOOT_UNDER, DONE}
    CheckState state = CheckState.SHOOTING;

    protected Limelight3A limelight;
    protected int blueTags;
    protected double seenStart = -1;
    protected double stateStart;

    public Pose turnPose;
    public Path toTurnPoint;//Back out of the corner area, still facing 0
    public Path toOffWall;//Turn back to 270 on the short slide, the reverse of turnSideways
    public Path toStart;//Back onto the start pose
    public Path start_to_park;//ShootFarUnderHive path
    public Path start_to_under;
    public Path under_to_park;

    @Override
    public void init(){
        //4 pickup init: hardware, its paths, and the follower on StartFarLeft facing 270
        super.init();
        limelight = addLimelight(hardwareMap, "LL");
        //Shorter than the 4 pickup so there is time left to check and shoot again
        shootEndTime = 9;//4 pickup: 10
        cornerHoldTime = 1;//4 pickup: 2
        maxTimeBreakout = failsafeTime;

        Pose start = StartFarLeft.pose();
        Pose offWallPose = OffWall.pose();
        Pose turnPoint = TurnPoint.pose();
        Pose prePickup = PreLooseBallsPickup.pose();
        Pose nav1 = MidUnderHive.pose();
        Pose nav2 = Quadrant3Center.pose();
        Pose under = UnderShotPoint.pose();
        parkPose = ParkPoseB.pose();//Also used by the failsafe park
        turnPose = new Pose(under.x(), under.y(), turnedHeading);

        //Return to the start pose the same way the 4 pickup left it, turning only on the slide away from the wall
        toTurnPoint = path(line(prePickup, turnPoint)).constant(sideHeading);
        toOffWall = path(line(turnPoint, offWallPose)).constant(startHeading);
        toStart = path(line(offWallPose, start)).constant(startHeading);

        //Same as ShootFarUnderHive: under the hive, through quadrant 3, to park, still facing 270
        start_to_park = path(line(start, nav1), line(nav1, nav2), line(nav2, parkPose)).constant(startHeading);
        //Under the hive to the turn point, facing 270 so it never turns on the way
        start_to_under = path(line(start, nav1), line(nav1, under)).constant(startHeading);
        //Already turned around, keep facing 90 to park
        under_to_park = path(line(under, parkPose)).constant(turnedHeading);
    }

    @Override
    public void init_loop(){
        readHives();
        telemetry.addData("Limelight Status", limelight.isConnected() ? "CONNECTED" : "Connecting...");
        telemetry.update();
    }

    @Override
    public void start(){
        super.start();//Reset timers so time spent in init doesn't count
        state = CheckState.SHOOTING;
        seenStart = -1;
    }

    @Override
    public void loop(){
        //Read the hive every loop, MovmentMain.loop() sends the telemetry
        readHives();
        telemetry.addData("Check state: ", state);
        //Intake path breakout: if the slow drive into the corner (state INTO_CORNER while following) takes longer than
        //intakePathTimeout, stop waiting on it and go straight to the corner hold. pathTimer resets when the path starts
        if (state == CheckState.INTO_CORNER && pathState == pathingState.RETURNMOVETOPOINT && pathTimer.seconds() > intakePathTimeout) {
            pathState = returnState;
            telemetry.addLine("Intake path timed out, holding corner");
        }
        super.loop();
    }

    //Counts this loop's blue hive tags (front + back), ignores stale/invalid results
    public void readHives(){
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = (result != null && result.isValid()) ? result.getFiducialResults() : Collections.<LLResultTypes.FiducialResult>emptyList();
        blueTags = 0;
        for (LLResultTypes.FiducialResult fr : fiducials) {
            int id = fr.getFiducialId();
            if ((id >= frontMinId && id <= frontMaxId) || (id >= backMinId && id <= backMaxId)) {
                blueTags++;
            }
        }
        telemetry.addData("Blue hive tags seen: ", blueTags);
    }

    //Called when the second shot starts: push the failsafe past the end of the shot so it can't cut the shot short
    public void keepFailsafeClear(double now){
        maxTimeBreakout = Math.max(maxTimeBreakout, now + secondShootDuration + 0.5);
    }

    //Shortest angle between two headings in radians
    public static double angleDiff(double a, double b){
        return Math.atan2(Math.sin(a - b), Math.cos(a - b));
    }

    //pathLogic only runs in the LOOP state, so each case after a movePath starts once that path has finished
    @Override
    public void pathLogic(){
        double now = opmodeTimer.seconds();
        switch (state) {
            //---- Same as BlueStartFar4PickUp ----
            case SHOOTING:
                launch(shootSpeed, 980, 3, 1);
                if (now > shootEndTime) {
                    cutPower();
                    state = CheckState.OFF_WALL;
                    movePath(offWall, pathingState.LOOP);
                }
                break;
            case OFF_WALL:
                state = CheckState.TURN_SIDEWAYS;
                movePath(turnSideways, pathingState.LOOP);
                break;
            case TURN_SIDEWAYS:
                state = CheckState.TO_PICKUP;
                movePath(toPickup, pathingState.LOOP);
                break;
            case TO_PICKUP:
                //Intake stays on from here until the robot has backed out of the corner
                intake(-1);
                state = CheckState.INTO_CORNER;
                movePath(intoCorner, pathingState.LOOP);
                break;
            case INTO_CORNER:
                //In the corner, hold the pose while the intake runs
                follower.hold(cornerPose);
                holdTimer.reset();
                state = CheckState.HOLD_CORNER;
                break;
            case HOLD_CORNER:
                if (holdTimer.seconds() > cornerHoldTime) {
                    state = CheckState.BACK_OUT;
                    movePath(backOut, pathingState.LOOP);
                }
                break;
            //---- New: go back to the start pose and check for the hive ----
            case BACK_OUT:
                intake(0);
                //Spin up on the way back so the shot can start sooner
                launcherMotor.setVelocity(secondShootSpeed);
                state = CheckState.RETURN_TURN_POINT;
                movePath(toTurnPoint, pathingState.LOOP);
                break;
            case RETURN_TURN_POINT:
                state = CheckState.RETURN_OFF_WALL;
                movePath(toOffWall, pathingState.LOOP);
                break;
            case RETURN_OFF_WALL:
                state = CheckState.RETURN_START;
                movePath(toStart, pathingState.LOOP);
                break;
            case RETURN_START:
                //Back on the start pose, start looking
                stateStart = now;
                seenStart = -1;
                state = CheckState.CHECK;
                break;
            case CHECK:
                launcherMotor.setVelocity(secondShootSpeed);
                if (blueTags > 0) {
                    if (seenStart < 0) seenStart = now;
                }
                else {
                    seenStart = -1;
                }
                if (seenStart >= 0 && now - seenStart >= confirmSeconds) {
                    stateStart = now;
                    keepFailsafeClear(now);
                    state = CheckState.SHOOT_START;
                }
                else if (now - stateStart > checkTime) {
                    //Can't see the hive, drive under it (launcher keeps spinning)
                    state = CheckState.DRIVE_UNDER;
                    movePath(start_to_under, pathingState.LOOP);
                }
                break;
            case SHOOT_START:
                launch(secondShootSpeed, secondShootSpeed, 3, 1);
                if (now - stateStart > secondShootDuration) {
                    cutPower();
                    state = CheckState.DONE;
                    movePath(start_to_park, pathingState.PARK);
                }
                break;
            case DRIVE_UNDER:
                //Past the hive, turn around in place to face it
                follower.hold(turnPose);
                stateStart = now;
                state = CheckState.TURN_AROUND;
                break;
            case TURN_AROUND:
                launcherMotor.setVelocity(secondShootSpeed);
                double error = Math.abs(angleDiff(follower.pose().heading(), turnedHeading));
                telemetry.addData("Turn error (deg): ", "%.1f", Math.toDegrees(error));
                if (error < turnTolerance || now - stateStart > maxTurnTime) {
                    stateStart = now;
                    keepFailsafeClear(now);
                    state = CheckState.SHOOT_UNDER;
                }
                break;
            case SHOOT_UNDER:
                //Still holding turnPose while shooting
                launch(secondShootSpeed, secondShootSpeed, 3, 1);
                if (now - stateStart > secondShootDuration) {
                    cutPower();
                    state = CheckState.DONE;
                    movePath(under_to_park, pathingState.PARK);
                }
                break;
            case DONE:
                break;
        }
    }
}
