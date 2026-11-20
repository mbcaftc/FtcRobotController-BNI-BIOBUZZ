package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew.IntakeAutos;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;
import org.firstinspires.ftc.teamcode.BioBuzz.pedro.MainConstants;

import java.util.Collections;
import java.util.List;

public class IntakeAuto extends MovmentMain {
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
    public Pose secondShot;
    //Control points for parking navigation
    public Pose nav3;
    public Pose nav4;
    //Parking Pose
    public Pose parkingPose;


    public Path start_to_intake;
    public Path intake;
    public Path shootTwo;
    public Path shoot_to_park;


    //The time that the intake started holding the final pose
    public double intakePositionHoldingTime;
    //Amount of ms that intake holds
    public double intakeHoldDuration = 400;

    //How long each shot may run
    public double firstShotTime;
    public double secondShotTime;

    //Time that robot started second shooting (seconds)
    public double secondShotStart;

    //Desired turret angle at each shot
    public double firstShotAngle;
    public double secondShotAngle;

    //Each shot Velocity
    public double firstShotVelocity;
    public double secondShotVelocity;
    public double intakeSpeed = .5;

    //StateMachine init
    public enum autoState {PRESTART, START, INTAKE, HOLDINTAKE, NAVTOSHOOT, STARTSHOOT, SHOOTTWO, CENTERTURRET, PARK, END}
    public autoState current = autoState.START;


    public void buildPaths(){
        //Move from start Pose to intake pose, rotate headings halfway
        start_to_intake = path(
                line(startPose, nav1).constant(startPose),
                line(nav1, startIntake).constant(startIntake)
        );

        //Traverse pollen line at slower speed
        intake = path(line(startIntake, endIntake).constant(startIntake)
        /*this makes the path slower*/    .with(MainConstants.foresightConfig.maxPathSpeed.at(intakeSpeed)));
        
        //Return from intake to the shooting position
        shootTwo = path(line(endIntake, nav2).constant(endIntake),
                        line(nav2, secondShot).constant(secondShot));
        
        //Navigate from final shot to the parking position.
        shoot_to_park = path(line(secondShot, nav3).tangent(),
                             line(nav3, nav4).tangent(),
                             line(nav4, parkingPose).constant(parkingPose));

    }

    @Override
    public void pathLogic(){

        switch(current){
            //Launch Before moving
            case PRESTART:
                primeForLaunch();
                current = autoState.START;
                break;
            case START:
                turretAngle = firstShotAngle;
                burnerLaunch(firstShotVelocity, 2);
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
                    current = autoState.STARTSHOOT;
                    movePath(shootTwo);
                }
                break;
            
            case STARTSHOOT:
                //Store the time that the shooting starts
                secondShotStart = opmodeTimer.seconds();
                current = autoState.SHOOTTWO;
                primeForLaunch();
                break;
            
            case SHOOTTWO:
                burnerLaunch(secondShotVelocity, 2);
                if (opmodeTimer.seconds() - secondShotStart > secondShotTime) {
                    turretAngle = 0;
                    cutPower();
                    movePath(shoot_to_park);
                    current = autoState.PARK;
                }
                break;
        }
    }
}
