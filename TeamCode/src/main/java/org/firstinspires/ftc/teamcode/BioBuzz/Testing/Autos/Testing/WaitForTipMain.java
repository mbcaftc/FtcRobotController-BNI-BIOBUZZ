package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Autos.Testing;

import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addLimelight;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.AutoTemplates.TwoPathMain;

import java.util.Collections;
import java.util.List;

//Shared base for WaitForTipBlue and WaitForTipRed. Starts like PROTOTYPE_AUTO_CLOSE (delay, drive to shoot, spin up),
//then at the shoot pose waits until it is clear to fire. Fires anyway at overrideTime. Then parks.
//Only this alliance's tags are considered (same as AllianceTeleOP). Clear to fire:
//  - 4 or more of the alliance's tags are seen (fireTagCount, front + back combined) -> fire
//    (also checked during the start delay: if seen, skip the rest of the delay and fire on arrival)
//  - Only one of the alliance's hives is seen -> fire
//  - Both of the alliance's hives are seen -> fire if one is higher than the other (more than levelThreshold apart)
//No @Autonomous annotation, so it does not show up on the Driver Station by itself.
public abstract class WaitForTipMain extends TwoPathMain {
    //April Tag ID ranges for this alliance's hives (set by WaitForTipBlue / WaitForTipRed), same as AllianceTeleOP
    protected String allianceName;
    protected int frontMinId, frontMaxId;
    protected int backMinId, backMaxId;

    //Seconds to wait after start before moving (same as PROTOTYPE_AUTO_CLOSE)
    public double startDelay = 10;
    //Fire at this time even if not clear to fire
    public double overrideTime = 15;
    //How long to shoot for once shooting starts
    public double shootDuration = 7;
    //Always leave for park by this time (maxTimeBreakout only stops the robot, it does not drive the park path)
    public double parkByTime = 22;

    //How far apart (meters of camera space y offset) the 2 hives have to be to count as one higher. TUNE THIS on the field.
    public double levelThreshold = 0.05;
    //Fire if at least this many of this alliance's tags are seen (front + back combined)
    public int fireTagCount = 4;
    //Has to be clear to fire this long before firing (filters out flickering detections)
    public double confirmSeconds = 0.5;

    protected Limelight3A limelight;

    protected int FTags, BTags;
    protected double FYoffset, BYoffset;
    protected double clearStart = -1;

    protected boolean shooting = false;
    //Set when enough tags were seen during the start delay: fire right away at the shoot pose
    protected boolean fireOnArrival = false;
    protected String shootReason = "";
    protected double shootStartTime;

    public WaitForTipMain(String alliance, int frontMin, int frontMax, int backMin, int backMax) {
        allianceName = alliance;
        frontMinId = frontMin;
        frontMaxId = frontMax;
        backMinId = backMin;
        backMaxId = backMax;
    }

    //Taken from AllianceTeleOP.autoTarget(): average the camera space y offset of each of this alliance's hives, ignore stale/invalid results
    public void readHives() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = (result != null && result.isValid()) ? result.getFiducialResults() : Collections.<LLResultTypes.FiducialResult>emptyList();

        FTags = 0; BTags = 0;
        double FSum = 0, BSum = 0;
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            int id = fr.getFiducialId();
            double y = fr.getTargetPoseCameraSpace().getPosition().y;
            //Front Cluster
            if (id >= frontMinId && id <= frontMaxId) { FTags++; FSum += y; }
            //Back Cluster
            if (id >= backMinId && id <= backMaxId) { BTags++; BSum += y; }
        }
        FYoffset = FTags > 0 ? FSum / FTags : 0;
        BYoffset = BTags > 0 ? BSum / BTags : 0;

        telemetry.addData(allianceName + " Front: ", "Tags: %d, Yoffset: %.3f", FTags, FYoffset);
        telemetry.addData(allianceName + " Back: ", "Tags: %d, Yoffset: %.3f", BTags, BYoffset);
    }

    //Enough of this alliance's tags seen (front + back combined)
    public boolean enoughTags() {
        boolean enough = FTags + BTags >= fireTagCount;
        telemetry.addData(allianceName + " tags seen: ", "%d (fire at %d)%s", FTags + BTags, fireTagCount, enough ? " CLEAR" : "");
        return enough;
    }

    public boolean clearToFire() {
        if (enoughTags()) return true;
        if (FTags > 0 && BTags > 0) {
            //Both seen: fire if one is higher (lower y offset = higher hive, same as AllianceTeleOP)
            double diff = Math.abs(FYoffset - BYoffset);
            telemetry.addData("Higher Hive: ", allianceName + (FYoffset < BYoffset ? " Front" : " Back"));
            telemetry.addData("Height difference: ", "%.3f (fire at %.3f)", diff, levelThreshold);
            return diff > levelThreshold;
        }
        //Only one of this alliance's hives seen: fire
        return FTags > 0 || BTags > 0;
    }

    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        limelight = addLimelight(hardwareMap, "LL");
        //Set BlueMain points (BEFORE buildConstantPaths()), same as PROTOTYPE_AUTO_CLOSE
        startPose = StartCloseLeft.pose();
        shootPose = Quadrant3Back.pose();
        parkPose = ParkPoseB.pose();
        //BlueMain init
        buildPaths();
        //Start the follower
        follower.setPose(new Pose(startPose.x(), startPose.y(), Math.toRadians(90)));
        follower.update();
    }

    @Override
    public void init_loop(){
        readHives();
        telemetry.addData("Limelight Status", limelight.isConnected() ? "CONNECTED" : "Connecting... (Ensure Panels is closed)");
        telemetry.update();
    }

    @Override
    public void loop(){
        //Read the hives every loop, MovmentMain.loop() sends the telemetry
        readHives();
        super.loop();
    }

    @Override
    public void pathLogic(){
        double now = opmodeTimer.seconds();
        if (moveToPointChain.equals(start_to_shoot)) {
            if (!shooting) {
                //At the shoot pose, keep the launcher spun up until clear to fire or the override time hits
                launcherMotor.setVelocity(1100);
                if (clearToFire()) {
                    if (clearStart < 0) clearStart = now;
                }
                else {
                    clearStart = -1;
                }
                telemetry.addData("Waiting to fire, override in: ", "%.1f", Math.max(0, overrideTime - now));

                boolean confirmed = clearStart >= 0 && now - clearStart >= confirmSeconds;
                if (fireOnArrival || confirmed || now >= overrideTime) {
                    shooting = true;
                    shootReason = fireOnArrival ? fireTagCount + " tags seen before leaving" : confirmed ? "Tags" : "Override";
                    shootStartTime = now;
                    shotTimer.reset();
                }
            }
            else {
                launch(1050, 1500, 3, 1);
                telemetry.addData("Shooting, reason: ", shootReason);
                if (now > shootStartTime + shootDuration || now > parkByTime) {
                    cutPower();
                    movePath(shoot_to_park, pathingState.PARK);
                }
            }
        }
        else {
            //During the start delay, leave early if enough of this alliance's tags are seen (held for confirmSeconds)
            if (enoughTags()) {
                if (clearStart < 0) clearStart = now;
            }
            else {
                clearStart = -1;
            }
            boolean tagsConfirmed = clearStart >= 0 && now - clearStart >= confirmSeconds;
            telemetry.addData("Start delay, leaving in: ", "%.1f", Math.max(0, startDelay - now));

            if (tagsConfirmed || now >= startDelay) {
                //Tags already confirmed, so fire as soon as the robot reaches the shoot pose
                fireOnArrival = tagsConfirmed;
                clearStart = -1;
                //Start the first path of start_to_shoot
                movePath(start_to_shoot, pathingState.LOOP);
                //Spin up launcher early
                launcherMotor.setVelocity(1100);
            }
        }
    }
}
