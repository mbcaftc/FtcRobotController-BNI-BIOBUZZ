package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;
import org.firstinspires.ftc.teamcode.BioBuzz.pedro.MainConstants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "IntakeAuto", group = "Debug")
public class IntakeAuto extends MovmentMain {
    //Starting Position
    public Pose startPose = StartFarLeft.setDeg(270);
    //Navigate to the intake position
    public Pose nav1 = TurnPoint.setDeg(0);
    //Start intake position
    public Pose startIntake = PreLooseBallsPickup.setDeg(0);
    //End of intake position
    public Pose endIntake = LooseBallsPickup.setDeg(0);
    //Control point for returning to shooting pose
    public Pose nav2 = TurnPoint.setDeg(180);
    //Second shooting pose
    public Pose secondShot = StartFarLeft.setDeg(270);
    //Control points for parking navigation
    public Pose nav3 = MidUnderHive.setDeg(270);
    public Pose nav4 = Quadrant3Center.setDeg(270);
    //Parking Pose
    public Pose parkingPose = ParkPoseB.setDeg(180);


    public Path start_to_intake;
    public Path intake;
    public Path shootTwo;
    public Path shoot_to_park;


    //The time that the intake started holding the final pose
    public double intakePositionHoldingTime;
    //Amount of ms that intake holds
    public double intakeHoldDuration = 400;

    //How long each shot may run
    public double firstShotTime = 7.5;
    public double secondShotTime = 7.5;

    //Time that robot started second shooting (seconds)
    public double secondShotStart;

    //Desired turret angle at each shot
    public double firstShotAngle = 0;
    public double secondShotAngle = 0;

    public double intakeSpeed = .6;

    //StateMachine init
    public enum autoState {START, INTAKE, HOLDINTAKE, NAVTOSHOOT, STARTSHOOT, SHOOTTWO, CENTERTURRET, PARK, END}
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
            case START:
                turretAngle = firstShotAngle;
                launch();
                if (opmodeTimer.seconds() > firstShotTime) {
                    turretAngle = 0;
                    cutPower();
                    launcherMotor.setVelocity(970);
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
                break;
            
            case SHOOTTWO:
                launch();
                if (opmodeTimer.seconds() - secondShotStart > secondShotTime) {
                    turretAngle = 0;
                    cutPower();
                    movePath(shoot_to_park);
                    current = autoState.PARK;
                }
                break;
        }
    }



    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        //Set the breakout time to 29s because auto is long
        maxTimeBreakout = 29;
        //Set up the paths
        buildPaths();
        //Start the follower
        follower.setPose(startPose);
        follower.update();
    }  

}
