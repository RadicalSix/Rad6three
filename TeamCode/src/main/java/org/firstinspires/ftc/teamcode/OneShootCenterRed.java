package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**

  This works

 */
//LinearOpMode
    //started 12/19 by Justin

@Autonomous(name = "OneShootCenterRed", group = "Auto")

public class OneShootCenterRed extends LinearOpMode {

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


    /*
        public OneShootCenterRed(){

        }
    */
    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);

        double startPosR = robot.MotorL.getCurrentPosition();
        robot.LiftServo.setPosition(.05);
        robot.ShotFeeder.setPosition(.9);
        robot.ConveyorServo.setPosition(0);//in
        robot.PressServoR.setPosition(1);//in
        robot.PressServoL.setPosition(0);//in
        robot.TouchServo.setPosition(1);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //shoot first ball then move servo so it can help hold up the ball

        status = "Start, move servo";
        telemetry.addData("Status:", status);
        telemetry.update();


        startPosR = robot.MotorL.getCurrentPosition();
        status = "drive back until white line";
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() > startPosR - 4200 && !doneDrive1) {//stop if hit line or go certain distance
            if (robot.ColSensor.blue() > 8) {
                doneDrive1 = true;//hit white line
            }
            robot.MotorL.setPower(-.55 * vl);
            robot.MotorR.setPower(-.55 * vr);
            robot.TouchServo.setPosition(.65);
            robot.PressServoL.setPosition(0);//in
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL current", robot.MotorL.getCurrentPosition() - startPosR);
            telemetry.addData("sensorColor:", robot.ColSensor.blue());
            telemetry.update();
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "drive back past line";
        startPosR = robot.MotorL.getCurrentPosition();
        robot.MotorL.setPower(-.3 * vl);
        robot.MotorR.setPower(-.3 * vr);
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() > startPosR - 650) && doneDrive1) {//only does this step if hit white line
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL units to go", robot.MotorL.getCurrentPosition() - startPosR + 650);
            telemetry.addData("MotorL current", robot.MotorL.getCurrentPosition());
            telemetry.update();
            idle();
        }

        status = "turn until white line";
        runtime.reset();
        turnOneCount = 0;
        lastPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.ColSensor.blue() < 8) {
            telemetry.addData("Status:", status);
            telemetry.update();
            robot.MotorL.setPower(-.33 * vl);
            robot.MotorR.setPower(.33 * vr);
            /*if(lastPosL == robot.MotorL.getCurrentPosition()) {
                turnOneCount++;
            }
            else if(lastPosL != robot.MotorL.getCurrentPosition()) {
                turnOneCount = 0;
            }
            if(turnOneCount > 25){
                while(robot.MotorL.getCurrentPosition() < startPosR + 70) {
                    robot.MotorL.setPower(.25);
                    robot.MotorR.setPower(.25);
                }
                turnOneCount = 0;
            }
            if(runtime.seconds() > 1){
                if(lastPosL == robot.MotorL.getCurrentPosition()){
                    while((robot.MotorL.getCurrentPosition() < (startPosR + 600)) && robot.ColSensor.blue() < 8) {
                        robot.MotorL.setPower(.5);
                        robot.MotorR.setPower(.5);
                    }
                }
                lastPosL = robot.MotorL.getCurrentPosition();
                runtime.reset();
            }*/
            //lastPosL = robot.MotorL.getCurrentPosition();
            telemetry.addData("lastPosL", lastPosL);
            telemetry.addData("current", robot.MotorL.getCurrentPosition());
            telemetry.addData("turnOneCount", turnOneCount);
        }

        status = "line follow";
        startPosR = robot.MotorL.getCurrentPosition();
        runtime.reset();
        lastPosL = robot.MotorL.getCurrentPosition();
        twolastPosL = 0;
        while (opModeIsActive() && !robot.TouSensor.isPressed()) {
            telemetry.addData("Status:", status);
            telemetry.addData("TouSensor.isPressed()", robot.TouSensor.isPressed());
            telemetry.update();
            if (robot.ColSensor.blue() < 8) {//grey
                robot.MotorR.setPower(.2 * vl);
                robot.MotorL.setPower(-.4 * vr);
            } else if (robot.ColSensor.blue() > 8) {//white
                robot.MotorR.setPower(-.4 * vl);
                robot.MotorL.setPower(.2 * vr);
            }
            if (runtime.seconds() > 3) {
                telemetry.addData("Status:", "Stuck");
                if (Math.abs(lastPosL - robot.MotorL.getCurrentPosition()) < 10) {
                    if (robot.ColSensor.blue() > 8) {
                        startPosR = robot.MotorL.getCurrentPosition();
                        while (robot.MotorL.getCurrentPosition() < (startPosR + 100 + (followOneCount * 50))) {
                            robot.MotorL.setPower(.5);
                            robot.MotorR.setPower(.25);
                        }
                        robot.MotorL.setPower(0);
                        robot.MotorR.setPower(0);
                    } else if (robot.ColSensor.blue() < 8) {
                        startPosR = robot.MotorL.getCurrentPosition();
                        while (robot.MotorL.getCurrentPosition() < (startPosR + 250 + (followOneCount * 50))) {
                            robot.MotorL.setPower(.25);
                            robot.MotorR.setPower(.5);
                        }
                        robot.MotorL.setPower(0);
                        robot.MotorR.setPower(0);
                    }
                }
                runtime.reset();
                lastPosL = robot.MotorL.getCurrentPosition();
                followOneCount++;
            }
            telemetry.addData("Time", runtime.seconds());
            telemetry.addData(" Change in last and current", Math.abs(lastPosL - robot.MotorL.getCurrentPosition()));
            telemetry.addData("followOneCount", followOneCount);
        }

        status = "sense color";
        telemetry.update();
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);
        startPosR = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .1) {
            telemetry.addData("Status:", status);
            telemetry.addData("sensorColor:", robot.ColSensor.blue());
            telemetry.update();
            if (robot.FruitySensor.blue() > robot.FruitySensor.red()) {
                beaconOneRed = false;
            } else {
                beaconOneRed = true;
            }
        }

        status = "forward off beacon 1";
        telemetry.update();
        startPosR = robot.MotorL.getCurrentPosition();
        robot.MotorL.setPower(.7 * vl);
        robot.MotorR.setPower(.7 * vr);
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosR + 225) {
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosR - 225);
            telemetry.update();
        }

        status = "turn paddles";
        telemetry.update();
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);
        robot.TouchServo.setPosition(0);
        startPosR = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1) {
            telemetry.addData("Status:", status);
            telemetry.update();
            if (beaconOneRed) {
                robot.PressServoR.setPosition(0);//out
            } else {
                robot.PressServoL.setPosition(.8);//in
            }
        }

        status = "backward and press buttons, beacon 1";
        startPosR = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() > startPosR - 330) && !beaconOneDone) {
            if (beaconOneRed) {
                robot.MotorL.setPower(-.7 * vl);
                robot.MotorR.setPower(-.3 * vr);
            } else {
                robot.MotorL.setPower(-.3 * vl);
                robot.MotorR.setPower(-.7 * vr);
            }
                /*if(lastPosL == robot.MotorL.getCurrentPosition()){
                    beaconOneCount++;
                }
                if(beaconOneCount > 10){
                    beaconOneDone = true;// stuck on wall
                }*/
            telemetry.addData("Status:", status);
            telemetry.addData("currentPos - startPosR + 400", robot.MotorL.getCurrentPosition() - startPosR + 330);
            telemetry.addData("beaconOneCount", beaconOneCount);
            telemetry.addData("beaconOneDone", beaconOneDone);
            telemetry.update();
            lastPosL = robot.MotorL.getCurrentPosition();
        }


        status = "forward off beacon 1";
        telemetry.update();
        startPosR = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosR + 1550) {
            robot.MotorL.setPower(.6 * vl);
            robot.MotorR.setPower(.6 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosR - 1550);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "start shooter";
        shot = 0;
        while (opModeIsActive() && shot < shotSpeed) {
            shot += 0.02;
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
            telemetry.addData("shot", shot);
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.5) {
            shot = shotSpeed;
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
            telemetry.addData("shot", shot);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        status = "shoot first ball";
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            shot = shotSpeed;
            robot.ShotFeeder.setPosition(0);
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
            telemetry.addData("shot", shot);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        status = "feed second ball";
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            robot.PressServoL.setPosition(.6);//out part of the way
            robot.TouchServo.setPosition(0);
            robot.ShotFeeder.setPosition(.9);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        status = "shoot second ball";
        robot.Conveyor.setPower(0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2.5) {
            shot = shotSpeed;
            robot.ShotFeeder.setPosition(0);
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
            telemetry.addData("shot", shot);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        while (opModeIsActive() && shot > 0.18) {
            shot -= 0.01;
            robot.ShotFeeder.setPosition(.9);
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
            telemetry.addData("shot", shot);
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        while (opModeIsActive() && shot > 0.0025) {
            shot -= 0.0025;
            robot.ShotFeeder.setPosition(.9);
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
            telemetry.addData("shot", shot);
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        status = "drive to center vortex, knock off cap ball";
        telemetry.update();
        startPosR = robot.MotorL.getCurrentPosition();
        robot.MotorL.setPower(.7 * vl);
        robot.MotorR.setPower(.7 * vr);
        robot.ShooterDown.setPower(0);
        robot.ShooterUp.setPower(0);
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosR + 4000) {
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosR - 4000);
            telemetry.update();
        }


    }
}


