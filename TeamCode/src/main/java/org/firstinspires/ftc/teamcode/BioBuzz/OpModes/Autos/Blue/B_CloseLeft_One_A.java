package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.AutoMain;
import org.firstinspires.ftc.teamcode.BioBuzz.pedro.MainConstants;

@Autonomous(name = "B_CloseLeft_One_A", group = "Autonomous")
public class B_CloseLeft_One_A extends AutoMain {

    protected static final PoseFactory poseFactory = PoseFactory.degrees();

    protected static Pose StartCloseLeft = new Pose(83, 8.8);//1
    protected static  Pose StartCloseRight = new Pose(106, 8.8);//2
    protected static  Pose MidUnderHive = new Pose(84, 72);//3

    protected static  Pose MidAroundCloseHive = new Pose(84, 72);//this is close to flower//4
    protected static  Pose MidAroundCloseHumans = new Pose (124, 72 );//this is close to humans//5
    protected static  Pose StartFarLeft = new Pose(83, 132);//6
    protected static final Pose StartRightRight = new Pose(106, 132);//7

    protected static final Pose ParkPoseA = new Pose(124, 72);//11
    protected static final Pose ParkPoseB = new Pose(124, 72);//12
    protected static final Pose LooseBallsPickup =  new Pose(132, 133);//13
    protected static final Pose Quadrant1Center = new Pose(108, 108);//15
    protected static final Pose Quadrant2Center = new Pose(72, 108);//15
    protected static final Pose Quadrant3Center = new Pose(72, 36);//16
    protected static final Pose Quadrant4Center = new Pose(108, 36);//17
    protected static final Pose FlowerClosePos = new Pose(95, 8.8);//18
    protected static final Pose FlowerCloseAround = new Pose(95, 18);//19
    protected static final Pose FlowerFarPos = new Pose(132, 94);//22
    protected static final Pose FlowerFarAround = new Pose(122, 94);//22
    protected static final Pose FlowerCloseWest= new Pose(80, 8.8);//18
    protected static final Pose FlowerCloseEast = new Pose(115, 8.8 );//19
    protected static final Pose FlowerFarNorth = new Pose(132, 94);//22
    protected static final Pose FlowerFarSouth = new Pose(122, 94);//22



    public Path StartToShoot;
    public Path ShootToPark;
    private enum AutoState {
        START_TO_SHOOT,
        SHOOTING,
        SHOOT_TO_PARK,
        PARKED
    }
    private AutoState currentState = AutoState.START_TO_SHOOT;

    private Follower follower;
    @Override
    public void init(){
        follower = MainConstants.createFollower(hardwareMap);

        follower.setPose(new Pose(StartCloseLeft.x(), StartCloseLeft.y(), 90));
        StartToShoot = createLinearPath(StartCloseLeft, Quadrant3Center, 90, 90);
        ShootToPark = createLinearPath(Quadrant3Center, ParkPoseA, 90, 0);

    }
    @Override
    public void start() {
        // Start running the first path immediately when the match starts
        follower.follow(StartToShoot);
        currentState = AutoState.START_TO_SHOOT;
    }
    @Override
    public void loop(){
        follower.update();

        switch (currentState) {
            case START_TO_SHOOT:
                if (!follower.isBusy()) {
                    currentState = AutoState.SHOOTING;
                }
                break;

            case SHOOTING:
                if (!false){
                    follower.follow(ShootToPark);
                    currentState = AutoState.SHOOT_TO_PARK;
                }
                break;

            case SHOOT_TO_PARK:
                if (!follower.isBusy()) {
                    currentState = AutoState.PARKED;
                }
                break;

            case PARKED:
                break;
        }

    }

}
