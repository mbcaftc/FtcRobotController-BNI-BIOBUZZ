package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue.Testing;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;
import com.pedropathing.paths.Path;

@Autonomous(name = "Start_Far_PickUp_oz", group = "Drive")
public class BlueStartFar4PickUp extends MovmentMain {
    protected double PreDelay = 0; //time before anything happens
    protected double ShootingTime = 0; //tme in opmode where shooting ends


    private ScoringState currentState;

    private enum ScoringState {

        Pre,
        WAITING,
        SHOOTING,
        GOING_TO_PICKUP,
        START_INTAKE,
        STOP_INTAKE,
        PARK
    }

    private Path prePickup;
    private Path pickup;
    private Path park;
    protected double turretStartAngle;

    @Override
    public void init() {
        // AutoMain setup routines
        setup();
        initHardWare();

        Pose start = new Pose(StartFarLeft.x(), StartFarLeft.y(), Math.toRadians(0));
        prePickup = buildPaths(StartFarLeft, PreLooseBallsPickup);
        pickup = buildPaths(PreLooseBallsPickup.withHeading(Math.toRadians(0)), LooseBallsPickup.withHeading(Math.toRadians(0)), PreLooseBallsPickup.withHeading(Math.toRadians(0)));
        park = buildPaths(PreLooseBallsPickup.withHeading(Math.toRadians(0)), Quadrant1Center.withHeading(Math.toRadians(90)), ParkPoseA.withHeading(Math.toRadians(180)));

        turretStartAngle = 0.0;
        follower.setPose(new Pose(start.x(), start.y(), start.heading()));
        follower.update();
        currentState = ScoringState.Pre;
    }

    @Override
    public void start() {
        currentState = ScoringState.WAITING;
    }

    @Override
    public void pathLogic() {
        switch (currentState) {
            case Pre:
                break;//this should never run
            case WAITING:
                if (opmodeTimer.seconds() > PreDelay) currentState = ScoringState.SHOOTING;
                break;

            case SHOOTING:
                launch(960, 1225, 3, 1);
                if (opmodeTimer.seconds() > ShootingTime) {
                    currentState = ScoringState.GOING_TO_PICKUP;
                }
                break;

            case GOING_TO_PICKUP:
                cutPower();
                movePath(prePickup, pathingState.LOOP);
                currentState = ScoringState.START_INTAKE;
                break;

            case START_INTAKE:
                intake(1);
                movePath(pickup, pathingState.LOOP);
                currentState = ScoringState.STOP_INTAKE;
                break;

            case STOP_INTAKE:
                intake(0);
                currentState = ScoringState.PARK;
                break;

            case PARK:
                movePath(park, pathingState.PARK);
                currentState = ScoringState.WAITING; // Protects from refreshing path instruction every loop frame
                break;
        }
    }
    @Override
    public void loop(){
        super.loop();
        telemetry.addData("Scoring State:", currentState);
    }
}
