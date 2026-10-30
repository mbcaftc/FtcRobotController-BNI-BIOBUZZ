package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.Range;

import java.util.Collections;
import java.util.List;

//Shared base for RedTeleOP and BlueTeleOP. Same as TeleOP, but only autotargets the hives of one alliance.
//No @TeleOp annotation, so it does not show up on the Driver Station by itself.
public abstract class AllianceTeleOP extends TeleOP {
    //April Tag ID ranges for this alliance's hives (set by RedTeleOP / BlueTeleOP)
    protected int frontMinId;
    protected int frontMaxId;
    protected int backMinId;
    protected int backMaxId;
    protected String allianceName;

    public AllianceTeleOP(boolean blue, int frontMin, int frontMax, int backMin, int backMax) {
        blueTeam = blue;
        allianceName = blue ? "Blue" : "Red";
        frontMinId = frontMin;
        frontMaxId = frontMax;
        backMinId = backMin;
        backMaxId = backMax;
    }

    @Override
    public void autoTarget() {
        if (gamepad2.y) {
            result = limelight.getLatestResult();

            double FavgX = 0;
            double FYoffset = 0;
            int FTags = 0;

            double BavgX = 0;
            double BYoffset = 0;
            int BTags = 0;

            //Average all tags positions that are within range, ignore stale/invalid results
            List<LLResultTypes.FiducialResult> fiducialResults = (result != null && result.isValid()) ? result.getFiducialResults() : Collections.<LLResultTypes.FiducialResult>emptyList();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                int id = fr.getFiducialId();
                //Front Cluster
                if (id >= frontMinId && id <= frontMaxId) {
                    FTags++;
                    FavgX += fr.getTargetXDegrees();
                    FYoffset += fr.getTargetPoseCameraSpace().getPosition().y;
                }
                //Back Cluster
                if (id >= backMinId && id <= backMaxId) {
                    BTags++;
                    BavgX += fr.getTargetXDegrees();
                    BYoffset += fr.getTargetPoseCameraSpace().getPosition().y;
                }
            }

            //Check if any tags are being seen
            if (FTags > 0 || BTags > 0) {
                //Average all values based on how many tags were detected
                if (FTags > 0) { FavgX /= FTags; FYoffset /= FTags; }
                if (BTags > 0) { BavgX /= BTags; BYoffset /= BTags; }

                //Take the lower of the 2 y values as the higher hive, if only one hive is seen target that one
                boolean front = FTags > 0 && (BTags == 0 || FYoffset < BYoffset);

                telemetry.addData("Target Hive: ", allianceName + (front ? " Front" : " Back"));
                telemetry.addData(allianceName + " Front: ", "Tags: %d, X: %.2f, Yoffset: %.2f", FTags, FavgX, FYoffset);
                telemetry.addData(allianceName + " Back: ", "Tags: %d, X: %.2f, Yoffset: %.2f", BTags, BavgX, BYoffset);

                double avgAng = front ? FavgX : BavgX;
                telemetry.addData("Avg Ang: ", avgAng);

                //Move according to midpoint of hive
                if (Math.abs(avgAng) > autoVariation) {
                    // Power scales with error, plus enough to overcome friction, capped to limit momentum
                    double power = kP * avgAng + Math.signum(avgAng) * kStatic;
                    power = Range.clip(power, -maxTurretPower, maxTurretPower);

                    axonIncrementAmount = turretDirection * power;
                    LEDCon(led, Math.abs(avgAng) > 6.0 ? 6 : 5);
                } else {
                    // Inside target window
                    axonIncrementAmount = 0.0;
                    LEDCon(led, 4);
                }
            }
            else {
                //Recenter the turret to 0 degrees(straight ahead), keep the sign so it turns back toward center
                double avgAng = turretCurrentHeading;
                if (Math.abs(avgAng) > autoVariation) {
                    double power = kP * avgAng + Math.signum(avgAng) * kStatic;
                    power = Range.clip(power, -maxTurretPower, maxTurretPower);

                    // Drive opposite the heading error
                    axonIncrementAmount = -turretHeadingSign * power;
                    LEDCon(led, 1);
                } else {
                    // Inside target window
                    axonIncrementAmount = 0.0;
                    LEDCon(led, 4);
                }
            }
            telemetry.addData("Axon Increment Amount: ", axonIncrementAmount);
        }
    }
}
