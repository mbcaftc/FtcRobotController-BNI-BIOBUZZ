package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew.IntakeAutos;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;
import org.firstinspires.ftc.teamcode.BioBuzz.pedro.MainConstants;

import java.util.Collections;
import java.util.List;

public class IntakeCheckHiveAutoNoTurn extends MovmentMain {
    //Starting Position
    public Pose startPose;
    //Navigate to the intake position
    public Pose nav1;
    //Start intake position
    public Pose startIntake;
    //End of intake position
    public Pose endIntake;
    //Control point for returning to shooting pose
    public Pose nav2;
    //Second shooting pose
    public Pose secondShotFar;
    public Pose secondShotClose;

    public Pose secondShotCloseNav1;
    //Control points for parking navigation
    public Pose nav3Far;
    public Pose nav4Far;

    public Pose returnNavUnderHive;
    public Pose nav3Close;
    //Parking Pose
    public Pose parkingPose;

    //Using blue tags?
    public boolean blueSide;

    //Limelight data
    protected LLResult result;

    public Path start_to_intake;
    public Path intake;
    public Path shootTwo;
    public Path navUnderHive;
    public Path shootClose_to_park;
    public Path shootFar_to_park;


    //The time that the intake started holding the final pose
    public double intakePositionHoldingTime;
    //Amount of ms that intake holds
    public double intakeHoldDuration = 400;

    //How long each shot may run
    public double firstShotTime;
    public double secondShotTime;

    //Time that robot started second shooting (seconds) / detecting tags
    public double secondShotStart;
    public double tagDetectionStart = -1;

    //Tag detection vars
    private int timesDetected = 0;
    private double totalTagsDetected = 0;
    public double minTagAccuracy = 2;

    //Desired turret angle at each shot
    public double firstShotAngle;
    public double secondShotAngle;

    //Each shot Velocity
    public double firstShotVelocity;
    public double secondShotVelocity;
    public double intakeSpeed = .6;

    //Estimated hive tip status
    public boolean hiveTip;


    //StateMachine init
    public enum autoState {START, INTAKE, HOLDINTAKE, NAVTOSHOOT, STARTSHOOT, CHECKHIVES, SHOOTTWOATHIVE, DRIVEUNDERHIVE, SHOOTTWOATCLOSE, CENTERTURRET, PARK, END}
    public autoState current = autoState.START;
    public autoState shootState;

    public void buildPaths(){
        //Move from start Pose to intake pose, rotate headings halfway
        start_to_intake = path(
                line(startPose, startIntake).constant(startIntake)
        );

        //Traverse pollen line at slower speed
        intake = path(line(startIntake, endIntake).constant(startIntake)
        /*this makes the path slower*/    .with(MainConstants.foresightConfig.maxPathSpeed.at(intakeSpeed)));
        
        //Return from intake to the shooting position
        shootTwo = path(line(endIntake, secondShotFar).constant(endIntake));


        navUnderHive = path(line(secondShotFar, secondShotClose).constant(secondShotFar));

        //Navigate from final shot to the parking position.
        shootClose_to_park = path(line(secondShotClose, nav3Close).tangent(),
                line(nav3Close, parkingPose).constant(parkingPose));


        //Navigate from final shot to the parking position.
        shootFar_to_park = path(line(secondShotFar, nav3Far).tangent(),
                             line(nav3Far, nav4Far).tangent(),
                             line(nav4Far, parkingPose).constant(parkingPose));

    }

    @Override
    public void pathLogic(){

        switch(current){
            //Launch Before moving
            case START:
                turretAngle = firstShotAngle;
                launch(firstShotVelocity);
                if (opmodeTimer.seconds() > firstShotTime) {
                    turretAngle = 0;
                    cutPower();
                    launcherMotor.setVelocity(secondShotVelocity);
                    movePath(start_to_intake);
                    current = autoState.INTAKE;
                }
                break;

            case INTAKE:
                intake(-1);
                current = autoState.HOLDINTAKE;
                //Move along the intake path, breakout after 2s in case gets stuck
                movePath(intake, 2);
                break;
            
            case HOLDINTAKE:
                //Store the time that the intake reaches the final position
                intakePositionHoldingTime = opmodeTimer.milliseconds();
                current = autoState.NAVTOSHOOT;
                break;
            
            case NAVTOSHOOT:
                turretAngle = secondShotAngle;
                if(opmodeTimer.milliseconds() - intakePositionHoldingTime > intakeHoldDuration){
                    current = autoState.CHECKHIVES;
                    movePath(shootTwo);
                }
                break;

            case CHECKHIVES:
                if(tagDetectionStart == -1){
                    tagDetectionStart = opmodeTimer.seconds();
                }
                //Delay 1 second to detect tags
                if(opmodeTimer.seconds() - tagDetectionStart < 1){
                    checkHive();
                }
                else{
                    //Check if the tag detection rate is larger than window
                    current = (totalTagsDetected / timesDetected > minTagAccuracy) ? autoState.STARTSHOOT : autoState.DRIVEUNDERHIVE;
                }
                break;


            case DRIVEUNDERHIVE:
                if(moveToPointChain.equals(navUnderHive)){
                    //Store the time that the shooting starts
                    secondShotStart = opmodeTimer.seconds();
                    current = autoState.SHOOTTWOATCLOSE;
                }
                else{
                    movePath(navUnderHive);
                }
                break;

            case STARTSHOOT:
                //Store the time that the shooting starts
                secondShotStart = opmodeTimer.seconds();
                current = autoState.SHOOTTWOATHIVE;
                break;
            
            case SHOOTTWOATHIVE:
                launch(secondShotVelocity);
                if (opmodeTimer.seconds() - secondShotStart > secondShotTime) {
                    turretAngle = 0;
                    cutPower();
                    movePath(shootFar_to_park);
                    current = autoState.PARK;
                }
                break;

            case SHOOTTWOATCLOSE:
                launch(secondShotVelocity);
                if (opmodeTimer.seconds() - secondShotStart > secondShotTime) {
                    turretAngle = 0;
                    cutPower();
                    movePath(shootClose_to_park);
                    current = autoState.PARK;
                }
                break;
        }
    }


    public void checkHive() {
        result = limelight.getLatestResult();
        int Tags = 0;
        //Average all tags positions that are within range, ignore stale/invalid results
        List<LLResultTypes.FiducialResult> fiducialResults = (result != null && result.isValid()) ? result.getFiducialResults() : Collections.<LLResultTypes.FiducialResult>emptyList();

        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            //RedFront Cluster
            if (!blueSide && fr.getFiducialId() >= 34 && fr.getFiducialId() <= 37) {
                Tags++;
            }
            //BlueBack Cluster
            if (blueSide && fr.getFiducialId() >= 42 && fr.getFiducialId() <= 45) {
                Tags++;
            }
        }

        //Add 1 to the times tags have been detected
        timesDetected++;

        //Check if any tags are being seen
        if (Tags > 0) {
            telemetry.addData("Target Hive: ", blueSide ? "Blue Back" : "Red Front");
            telemetry.addData("Tags Seen: ", Tags);
            totalTagsDetected += Tags;
        }
    }
}
