package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew.IntakeAutos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;

import java.util.Collections;
import java.util.List;

public class CloseCheckInstant extends MovmentMain {
    //Starting Position
    public Pose startPose;
    public Pose shootingPose;
    //Parking Pose
    public Pose parkingPose;

    //Limelight data
    protected LLResult result;

    public Path start_to_park;

    public double shotTime;
    public double initialDelay;

    //Time that robot started second shooting (seconds) / detecting tags
    public double shotStart;
    public double tagDetectionStart = -1;

    //Tag detection vars
    private int timesDetected = 0;
    private double totalTagsDetected = 0;
    public double minTagAccuracy = 2;

    //Desired turret angle at each shot
    public double shotAngle;

    //shot Velocity
    public double shotVelocity;

    //StateMachine init
    public enum autoState {START, STARTSHOOT, CHECKHIVES, SHOOTATHIVE, PARK}
    public autoState current = autoState.START;

    public void buildPaths(){
        //Move from start Pose
        start_to_park = path(
                curve(startPose, shootingPose, parkingPose).linear(startPose, parkingPose)
        );
    }

    @Override
    public void pathLogic(){
        telemetry.addData("Path state: ", current);
        switch(current){
            //Launch Before moving
            case START:
                current = autoState.CHECKHIVES;
                break;

            case CHECKHIVES:
                //Check the hives
                checkHive();
                //Check in 1s intervals if is good on tags
                if (tagDetectionStart <= opmodeTimer.seconds() - 1) {
                    //If tags seen, fire
                    if(totalTagsDetected / Math.max(timesDetected, 1) > minTagAccuracy){
                        launcherMotor.setVelocity(shotVelocity);
                        current = autoState.STARTSHOOT;
                        shotStart = opmodeTimer.seconds();
                    }
                    tagDetectionStart = opmodeTimer.seconds();
                    totalTagsDetected = 0;
                    timesDetected = 0;
                }
                //If we need to breakout, then do so
                if (opmodeTimer.seconds() - initialDelay > 0) {
                    current = autoState.PARK;
                }
                break;
                

            case STARTSHOOT:
                //Store the time that the shooting starts
                shotStart = opmodeTimer.seconds();
                primeForLaunch();
                current = autoState.SHOOTATHIVE;
                break;

            case SHOOTATHIVE:
                turretAngle = shotAngle;
                burnerLaunch(shotVelocity, 2);
                if (opmodeTimer.seconds() > shotTime + shotStart) {
                    cutPower();
                    current = autoState.PARK;
                    turretAngle = 0;

                }
                break;

            case PARK:
                cutPower();
                movePath(start_to_park, pathingState.PARK);
                break;
        }
    }


    public void checkHive() {
        result = limelight.getLatestResult();
        int Tags = 0;
        //Average all tags positions that are within range, ignore stale/invalid results
        List<LLResultTypes.FiducialResult> fiducialResults = (result != null && result.isValid()) ? result.getFiducialResults() : Collections.<LLResultTypes.FiducialResult>emptyList();

        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            //RedBack Cluster                                           BlueClose Cluster
            if (fr.getFiducialId() >= 30 && fr.getFiducialId() <= 33 || fr.getFiducialId() >= 38 && fr.getFiducialId() <= 41) {
                Tags++;
            }

        }
        //Add 1 to the times tags have been detected
        if(result != null && result.isValid()){timesDetected++;}


        //Check if any tags are being seen
        if (Tags > 0) {
            telemetry.addData("Tags Seen: ", Tags);
            totalTagsDetected += Tags;
        }
    }
}
