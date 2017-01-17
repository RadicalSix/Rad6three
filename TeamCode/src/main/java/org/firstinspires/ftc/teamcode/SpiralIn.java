package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * Created by Mitch on 1/3/17.
 */

public class SpiralIn extends LinearOpMode {

    HardwarePushbotTDR robot = new HardwarePushbotTDR();
    VuforiaOp camera = new VuforiaOp();
    private ElapsedTime runtime = new ElapsedTime();
    Boolean beaconOneRed;
    Boolean beaconTwoRed;
    Boolean doneDrive1 = false;
    Boolean beaconOneDone = false;
    Boolean beaconTwoDone = false;
    Boolean forwardTwoDone = false;
    Boolean turnTwoDone = false;
    Boolean longDriveDone = false;
    Boolean followOneDone = false;


    double vr = 1;//change for direction and battery
    double vl = 1;//change for direction and battery
    double shotSpeed = .39;//change for battery
    int step = 0;
    double shot = 0;
    double lastPosL = 0;
    double twolastPosL = 0;
    int beaconOneCount = 0;
    int BeaconTwoCount = 0;
    int forwardTwoCount = 0;
    int turnTwoCount = 0;
    int followOneCount = 0;
    int turnOneCount = 0;

    String status = "Start";

    public void runOpMode() throws InterruptedException {



        robot.init(hardwareMap);
        status = "turn until white line";
        runtime.reset();
        turnOneCount = 0;
        lastPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.ColSensor.blue() < 8) {
            telemetry.addData("Status:", status);
            telemetry.update();
            robot.MotorL.setPower(-.33 * vl);
            robot.MotorR.setPower(.33 * vr);
        }
    }
}