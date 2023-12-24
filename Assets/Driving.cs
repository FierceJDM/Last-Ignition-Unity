using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

public class Driving : MonoBehaviour {
    public List<AxleInfo> axleInfos; // the information about each individual axle
    public float maxMotorTorque; // maximum torque the motor can apply to wheel
    public float maxSteeringAngle; // maximum steer angle the wheel can have

    public void ApplyVisualsToWheelColliders(WheelCollider collider, float Zvalue)
    {
        if (collider.transform.childCount == 0) {
            return;
        }
     
        Transform visualWheel = collider.transform.GetChild(0);
     
        Vector3 position;
        Quaternion rotationQ;
        Vector3 rotationV3;
        collider.GetWorldPose(out position, out rotationQ); //get position and rotation of WheelCollider
        rotationV3 = rotationQ.eulerAngles; //convert to Vector3 to correct Z orientation
     
        visualWheel.transform.position = position;
        visualWheel.transform.rotation = Quaternion.Euler(rotationV3.x, rotationV3.y, Zvalue+rotationV3.z);
    }        



    public void Update()
    {
        Rigidbody RigidBody = GetComponent<Rigidbody>();
        float CurrentSpeed = RigidBody.velocity.magnitude * 2.0f;
        Debug.Log("Speed : " + CurrentSpeed);

        float motor = maxMotorTorque * Input.GetAxis("Vertical");
        float steering = maxSteeringAngle * Input.GetAxis("Horizontal");
        if (CurrentSpeed >= 1) {
            if (CurrentSpeed <= 15) {
                steering = ((maxSteeringAngle+10) - ((maxSteeringAngle+6)*(CurrentSpeed/15))) * Input.GetAxis("Horizontal");
            } else {
                steering = (maxSteeringAngle / (CurrentSpeed/4.0f)) * Input.GetAxis("Horizontal");
            }
        } // TODO : Tweak Steering values
            
        foreach (AxleInfo axleInfo in axleInfos) {
            if (axleInfo.steering) {
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
            if (axleInfo.motor) {
                axleInfo.leftWheel.motorTorque = motor;
                axleInfo.rightWheel.motorTorque = motor;
            }
            ApplyVisualsToWheelColliders(axleInfo.leftWheel, 180);
            ApplyVisualsToWheelColliders(axleInfo.rightWheel, 0);
        }
    }
}
    
[System.Serializable]
public class AxleInfo {
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor; // is this wheel attached to motor?
    public bool steering; // does this wheel apply steer angle?
}