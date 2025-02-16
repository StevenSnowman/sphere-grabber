#ifndef RescueTask_h
#define RescueTask_h

#include <Arduino.h>
#include "firmware/motor_handler.h"

enum Color {
    BLACK,
    SILVER,
    NONE
};

enum MotorResult{
    PASS,
    NOTPOSSIBLE,
    TIMEOUT,
    SHUTDOWN
};

struct JointPos{
    float radA1;
    float radA2;
    float radA3;
    bool valid;
};

struct Position{
    float targetX;
    float targetY;
};

class cameraHandler{
    int getTargetNum();
    int getCamDistance();
    Position bestTarget();
};

class Grabber{
    private:
    float minHorAngle; //rad
    float maxHorAngle;
    float minVerAngle;
    float maxVerAngle;
    float maxCameraAngle;

    float maxDynamicAngleError;
    float maxStaticAngleError;
    unsigned long timeOutMillisec;

    float armLength1;
    float armLength2;
    float armHeight;
    float ballRadius;

    float lockA1Kp;
    float lockA2Kp;
    float lockA3Kp;
    float poseK1p;
    float poseK2p;
    float poseK3p;
    float lockRotateKp;
    float lockLinearKP;
    

    public:
    Grabber(float minHorAngle, float maxHorAngle, float minVerAngle, float maxVerAngle, float maxDynamicAngleError, float maxStaticAngleError, float armLength1, float armLength2, float armHeight, float ballRadius, unsigned long timeOutMillisec);


    MotorResult moveTo(float x, float y, float z, float maxVelocity, MotorHandler motorControl); //Move to anywhere within the front cone constraint
    //return true if arrvied, false if the motor failed (overcurrent, stucked, rebooted, no solution, kinematic constrained, timeout)
    
    MotorResult moveToDefault();
    
    MotorResult sphereSearch(); // return true if found, false if target not found or arm failed
    MotorResult stationSearch(); //...

    MotorResult stickToTarget(float targetX, float targetY, float camDistance, MotorHandler motorControl); // return true if locked on, false if the target is lost or arm failed
    MotorResult stickToStation();

    Color grabTarget(); //return the grabbed color, NONE if nothing is grabbed or arm failed

    MotorResult loadStorage(Color color);

    JointPos inverseKinematicSolution(float x, float y, float z, float l1, float l2);

    double getCameraAngle(MotorHandler motorControl);
};


class Navigator{
    public:

    void enterRescueZone();
    void exitRescueZone();
    void relocate();
    void orientateToUnloadPosition();
};

class StorageManager{
    public:
    short blackSphereCount;
    short silverSphereCount;

    StorageManager();

    bool unloadPayload(Color color);
};


#endif