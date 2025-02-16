#include "RescueTask.h"
#include "firmware/motor_handler.h"

MotorResult Grabber::stickToTarget(float targetX, float targetY, float camDistance, MotorHandler mtrCtrl){
    //feedback controller, need large p gain with a limiter

    mtrCtrl.setControlMode(mtrCtrl.mtrA1, mtrCtrl.volCtrlID);
    mtrCtrl.setControlMode(mtrCtrl.mtrA2, mtrCtrl.volCtrlID);
    mtrCtrl.setControlMode(mtrCtrl.mtrA3, mtrCtrl.volCtrlID);

    //camera spring
    if(mtrCtrl.getPos(mtrCtrl.mtrA1) >= maxHorAngle || mtrCtrl.getPos(mtrCtrl.mtrA1) <= minHorAngle){
        mtrCtrl.setVol(mtrCtrl.mtrA1, 0);
        mtrCtrl.setRotateVol(lockRotateKp * targetX);
    } else {
        mtrCtrl.setVol(mtrCtrl.mtrA1, lockA1Kp * targetX);
    }
    
    mtrCtrl.setVol(mtrCtrl.mtrA3, lockA3Kp * targetY);
    
    //camera distance spring
    if (camDistance > 0 && mtrCtrl.getPos(mtrCtrl.mtrA3) > 0 ){
        mtrCtrl.setVol(mtrCtrl.mtrA2, lockA2Kp * camDistance);
    } else if(camDistance > 0 && mtrCtrl.getPos(mtrCtrl.mtrA3) < 0 ){
        mtrCtrl.setVol(mtrCtrl.mtrA2, -1 * lockA2Kp * camDistance);
    } else if (camDistance < 0){
        mtrCtrl.setVol(mtrCtrl.mtrA2, -1 * abs(lockA2Kp * camDistance));
    }

    //robot distance spring
    double goalRadialDist = armLength1 * cos(asin( armLength2 - (armHeight - 2 * ballRadius) / armLength1 ));

    double posTheta = mtrCtrl.getPos(mtrCtrl.mtrA1);
    double camAngle = acos((armLength1 * sin(mtrCtrl.getPos(mtrCtrl.mtrA2)) + armHeight - ballRadius) / (armLength2 + camDistance + ballRadius));
    double radialDist = (armLength2 + camDistance + ballRadius) * cos(PI / 2 - camAngle) + armLength1 * cos(mtrCtrl.getPos(mtrCtrl.mtrA2));
    
    double errorX = (radialDist * cos(mtrCtrl.getPos(mtrCtrl.mtrA1)) - goalRadialDist * cos(mtrCtrl.getPos(mtrCtrl.mtrA1)));
    double errorY = (radialDist * sin(mtrCtrl.getPos(mtrCtrl.mtrA1)) - goalRadialDist * sin(mtrCtrl.getPos(mtrCtrl.mtrA1)));
    
    double errorXRobotFrame = 0 - (errorX * cos(posTheta) + errorY * sin(posTheta));
    double errorYRobotFrame = 0 - (-1 * errorX * sin(posTheta) + errorY * cos(posTheta));

    mtrCtrl.setLinearVol(poseK1p * errorXRobotFrame);
    mtrCtrl.setRotateVol(poseK2p * errorYRobotFrame + poseK3p * posTheta);

    if(mtrCtrl.isShutdown(mtrCtrl.mtrA1) || mtrCtrl.isShutdown(mtrCtrl.mtrA2) || mtrCtrl.isShutdown(mtrCtrl.mtrA3)){
        return SHUTDOWN;
    }

    return PASS;
}

Color Grabber::grabTarget(){
    // stickToTarget with camDistance non deduced
    // close grabber
}

MotorResult Grabber::loadStorage(Color color){
    //moveTo(storage x, y)
    // open grabber
}

JointPos inverseKinematicSolution(double x, double y, double z, double l1, double l2){
    JointPos ans;

    //add a option that determins if you want to invert the arm, meaning the arm pass through the singulariry ( origin ) to reach the waypoint

    double r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

    if (r > l1 + l2 || r < fabs(l1 - l2)) {
        ans.valid = false;
        return ans;  // No solution
    }

    ans.radA1 = atan2(y , x);
    ans.radA3 = -1 * acos( (pow(l1, 2) + pow(l2, 2) - pow(r, 2)) / (2 * l1 * l2) );
    ans.radA2 = atan2(z , sqrt(pow(x, 2) + pow(y, 2))) + atan2( (l2 * sin(ans.radA3)) , (l1 + l2 * cos(ans.radA3)));

    ans.valid = true;

    return ans;
}

MotorResult Grabber::moveTo(float x, float y, float z, float maxVelocity, MotorHandler mtrCtrl){
    // target --> waypoints --> ik actuator pos --> actuator path (step) --> actuator pos control

    JointPos targetJointPos;
    targetJointPos = inverseKinematicSolution(x, y, z, armLength1, armLength2);


    /*
    if(targetJointPos.radA1 < minHorAngle || targetJointPos.radA1 > maxHorAngle){
        return NOTPOSSIBLE;
    } else if(targetJointPos.radA2 < minVerAngle || targetJointPos.radA2 > maxVerAngle){
        return NOTPOSSIBLE;
    } 
    */
   // safe operation area should be determind by the end point, not the traveling angle nor using min and max
   // not by traveling becuase the path to waypoint varies and is not always optimal ( not straight ) and the path does not ganernteen empty

    double curPos1 = mtrCtrl.getPos(mtrCtrl.mtrA1);
    double curPos2 = mtrCtrl.getPos(mtrCtrl.mtrA2);
    double curPos3 = mtrCtrl.getPos(mtrCtrl.mtrA3);

    double d1 = targetJointPos.radA1 - curPos1;
    double d2 = targetJointPos.radA2 - curPos2;
    double d3 = targetJointPos.radA3 - curPos3;

    double goalPost1 = curPos1;
    double goalPost2 = curPos2;
    double goalPost3 = curPos3;

    double totalError = sqrt(pow(d1, 2) + pow(d2, 2) + pow(d3, 2)); // Lyapunov function: if v' < 0 --> gradV dot f(x) < 0  (stable)
    unsigned long startTime = millis();
    unsigned long lastUpdateTime = millis();

    while(totalError > maxStaticAngleError){
        if(mtrCtrl.isShutdown(mtrCtrl.mtrA1) || mtrCtrl.isShutdown(mtrCtrl.mtrA2) || mtrCtrl.isShutdown(mtrCtrl.mtrA3)){
            return SHUTDOWN;
        }
        curPos1 = mtrCtrl.getPos(mtrCtrl.mtrA1);
        curPos2 = mtrCtrl.getPos(mtrCtrl.mtrA2);
        curPos3 = mtrCtrl.getPos(mtrCtrl.mtrA3);

        d1 = targetJointPos.radA1 - curPos1;
        d2 = targetJointPos.radA2 - curPos2;
        d3 = targetJointPos.radA3 - curPos3;

        totalError = sqrt(pow(d1, 2) + pow(d2, 2) + pow(d3, 2));
        
        double maxStep = maxVelocity * (millis() - lastUpdateTime) * 1000;

        if (abs(d1) < abs(maxStep)) {
            goalPost1 = targetJointPos.radA1;
        } else {
            goalPost1 = curPos1 + ((d1 > 0) ? 1 : -1) * maxStep;
        }

        if (abs(d2) < abs(maxStep)) {
            goalPost2 = targetJointPos.radA2;
        } else {
            goalPost2 = curPos2 + ((d2 > 0) ? 1 : -1) * maxStep;
        }

        if (abs(d3) < abs(maxStep)) {
            goalPost3 = targetJointPos.radA3;
        } else {
            goalPost3 = curPos3 + ((d3 > 0) ? 1 : -1) * maxStep;
        }


        mtrCtrl.setPos(mtrCtrl.mtrA1, goalPost1);
        mtrCtrl.setPos(mtrCtrl.mtrA2, goalPost2);
        mtrCtrl.setPos(mtrCtrl.mtrA3, goalPost3);
        lastUpdateTime = millis();

        if(millis() - startTime > timeOutMillisec){
            return TIMEOUT;
        }
    }
    
    return PASS;
}

MotorResult Grabber::sphereSearch(){
    float movementRadius = 10;
    // all viable ik solutions that interserts with this sphere from the origin --> literable list
    // search path camera min --> max (if identification > expected --> misitentification --> camera down)
    //moveTo(listx, listy)
    //break when identified target with confidence (move around and it is still there?)
    return PASS;
}

MotorResult Grabber::stationSearch(){
    //moveTo(listx, listy)
    //break when identified target with confidence (move around and it is still there?)
    return PASS;
}


void Navigator::enterRescueZone(){

}

void Navigator::exitRescueZone(){

}

void Navigator::relocate(){

}

void Navigator::orientateToUnloadPosition(){

}

bool StorageManager::unloadPayload(Color color){

}

StorageManager::StorageManager(){
    blackSphereCount = 0;
    silverSphereCount = 0;
}
