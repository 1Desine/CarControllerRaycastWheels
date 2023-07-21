// REFERENCE https://youtu.be/CdPYlj5uZeI or https://www.youtube.com/watch?v=CdPYlj5uZeI&t=13s&ab_channel=ToyfulGames

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Animations;

public class CarController : MonoBehaviour {

    [SerializeField] private Rigidbody carRigidBody;
    [SerializeField] private List<Transform> tireTransformList;

    [SerializeField] private List<Transform> tireToSteerTransformList;
    [SerializeField] private List<Transform> tireToApplyAccelerationForceTransformList;

    [SerializeField] private List<Transform> tireTransformVisualList;

    private float springStrength = 300f;
    private float springDamper = 5f;
    private float springDistance = 0.4f;
    private float suspentionRestDistance = 0.25f;

    [SerializeField] private AnimationCurve gripCurve;
    private float tireGripFactor = 0.7f;

    private float acceleration = 10f;
    private float carTopSpeed = 20f;
    [SerializeField] private AnimationCurve powerCurve;
    private float autoDecelerationForce = 0.03f;
    private float stoppingForce = 0.3f;

    private float tireRadiusVisual = 0.25f;

    [SerializeField] private AnimationCurve steerCurve;



    private float steerInputNormalized = 0f;
    private float throttleInputNormalized = 0f;


    private void Update() {
        HandleSteering();
        HandleTireVisual();


        if(Input.GetKey(KeyCode.E)) {
            transform.rotation = Quaternion.identity;
            transform.position += Vector3.up * 0.1f;
            carRigidBody.velocity = Vector3.zero;
            carRigidBody.angularVelocity = Vector3.zero;
        }
    }

    private void FixedUpdate() {
        HandleSuspention();
        HandleSideGrip();
        HandleAccelerationAndDeceleration();
    }

    private void HandleSteering() {
        float maxSteeringAngle = 45f;
        float steerSpeed = 20f * Time.deltaTime;

        if(Input.GetKey(KeyCode.A)) {
            steerInputNormalized = Mathf.Lerp(steerInputNormalized, -1, steerSpeed);
        }
        if(Input.GetKey(KeyCode.D)) {
            steerInputNormalized = Mathf.Lerp(steerInputNormalized, 1, steerSpeed);
        }
        if(!Input.GetKey(KeyCode.A) && !Input.GetKey(KeyCode.D)) {
            steerInputNormalized = Mathf.Lerp(steerInputNormalized, 0, steerSpeed / 2);
        }



        float carSpeed = Vector3.Dot(transform.forward, carRigidBody.velocity);
        float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);
        float steeringAfterSpeedCorrection = steerCurve.Evaluate(normalizedSpeed) * steerInputNormalized * maxSteeringAngle;

        Vector3 slideDirection = gameObject.transform.right;
        Vector3 carWorldVelocity = carRigidBody.velocity;
        float sideVelocity = Vector3.Dot(slideDirection, carWorldVelocity);

        float steeringCorrection = 1.25f;
        steeringAfterSpeedCorrection += sideVelocity * steeringCorrection;

        foreach(Transform tireTransform in tireToSteerTransformList) {
            tireTransform.localEulerAngles = new Vector3(0, steeringAfterSpeedCorrection, 0);
        }
    }

    private void HandleAccelerationAndDeceleration() {
        float throttleChangeSpeed = 10f;

        if(Input.GetKey(KeyCode.W)) {
            throttleInputNormalized = Mathf.Lerp(throttleInputNormalized, 1, throttleChangeSpeed);
        }
        if(Input.GetKey(KeyCode.S)) {
            throttleInputNormalized = Mathf.Lerp(throttleInputNormalized, -1, throttleChangeSpeed);
        }
        if(!Input.GetKey(KeyCode.W) && !Input.GetKey(KeyCode.S)) {
            throttleInputNormalized = Mathf.Lerp(throttleInputNormalized, 0, throttleChangeSpeed);
        }

        if(Input.GetKey(KeyCode.LeftShift)) {
            throttleInputNormalized *= 1.5f;
        }

        foreach(Transform tireTransform in tireToApplyAccelerationForceTransformList) {
            Vector3 accelerationDirection = tireTransform.forward;

            if(Physics.Raycast(tireTransform.position, -tireTransform.transform.up, out RaycastHit tireRay, springDistance)) {
                Vector3 tireDirection = tireTransform.forward;

                Vector3 tireWorldVelocity = carRigidBody.GetPointVelocity(tireTransform.position);

                float tireForwardVelocity = Vector3.Dot(tireDirection, tireWorldVelocity);
                // Auto deceleration
                if(throttleInputNormalized == 0) {
                    float desiredVelocityChange = -Mathf.Clamp(tireForwardVelocity * 1000, -1, 1) * autoDecelerationForce;

                    float desiredDeceleration = desiredVelocityChange / Time.fixedDeltaTime;

                    carRigidBody.AddForceAtPosition(tireDirection * desiredDeceleration, tireRay.point);
                } else {
                    float changeDirectionMargin = -1f;

                    // Stopping
                    if(throttleInputNormalized * tireForwardVelocity < changeDirectionMargin) {
                        float desiredVelocityChange = -Mathf.Clamp(tireForwardVelocity * 1000, -1, 1) * stoppingForce;

                        float desiredDeceleration = desiredVelocityChange / Time.fixedDeltaTime;

                        carRigidBody.AddForceAtPosition(tireDirection * desiredDeceleration, tireRay.point);
                    }

                    // Accelirating
                    if(throttleInputNormalized * tireForwardVelocity > changeDirectionMargin) {
                        float carSpeed = Vector3.Dot(transform.forward, carRigidBody.velocity);

                        float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);

                        float availableTorque = powerCurve.Evaluate(normalizedSpeed) * acceleration * throttleInputNormalized;

                        carRigidBody.AddForceAtPosition(accelerationDirection * availableTorque, tireRay.point);
                    }
                }
            }
        }
    }

    private void HandleSideGrip() {
        foreach(Transform tireTransform in tireTransformList) {
            if(Physics.Raycast(tireTransform.position, -tireTransform.transform.up, out RaycastHit tireRay, springDistance)) {
                // Direction to reduce sliding in
                Vector3 slideDirection = tireTransform.right;
                // World velocity of a tire - diection
                Vector3 tireWorldVelocity = carRigidBody.GetPointVelocity(tireTransform.position);

                // Getting velocity of "Direction to reduce sliding in"
                float tireSlidingVelocity = Vector3.Dot(slideDirection, tireWorldVelocity);

                float maxSlidingEffectingSpeed = 10f;

                // Modifying side grip based of sliding speed
                float tireSlidingGripNormalized = Mathf.Clamp01(Mathf.Abs(tireSlidingVelocity / maxSlidingEffectingSpeed));


                float dinamicGrip = gripCurve.Evaluate(tireSlidingGripNormalized) * tireGripFactor;




                // Counter velocity based om dinamic grip
                float maxCounterSlidingVelocity = 1f;
                float desiredVelocityChange = -tireSlidingVelocity * dinamicGrip;
                desiredVelocityChange = Mathf.Clamp(desiredVelocityChange, -maxCounterSlidingVelocity, maxCounterSlidingVelocity);

                // Acceliration = velocity / time
                float counterForce = desiredVelocityChange / Time.fixedDeltaTime;

                // Mass per tire
                float tireMass = 1f / tireTransformList.Count;

                // Add counter force. Force = mass * acceliration
                if(Input.GetKey(KeyCode.Space) && tireToApplyAccelerationForceTransformList.Contains(tireTransform)) {
                    // Power tire slipping
                    carRigidBody.AddForceAtPosition(slideDirection * tireMass * counterForce * 0.4f, tireRay.point);
                } else {
                    carRigidBody.AddForceAtPosition(slideDirection * tireMass * counterForce, tireRay.point);
                }


                Debug.DrawRay(tireTransform.position, tireWorldVelocity, Color.yellow);
                Debug.DrawRay(tireTransform.position, tireTransform.right * desiredVelocityChange, Color.blue);
            }
        }
    }

    private void HandleSuspention() {
        foreach(Transform tireTransform in tireTransformList) {
            Debug.DrawRay(tireTransform.position, -tireTransform.transform.up * springDistance, Color.black);

            if(Physics.Raycast(tireTransform.position, -tireTransform.transform.up, out RaycastHit tireRay, springDistance)) {
                Vector3 springDirection = tireTransform.up;

                Vector3 tireWorldVelocity = carRigidBody.GetPointVelocity(tireTransform.position);

                float offset = suspentionRestDistance - tireRay.distance;

                float velocity = Vector3.Dot(springDirection, tireWorldVelocity);

                float springStrengthPerWheel = springStrength / tireTransformList.Count;

                float force = (offset * springStrengthPerWheel) - (velocity * springDamper);

                carRigidBody.AddForceAtPosition(springDirection * force, tireTransform.position);
            }
        }
    }

    private void HandleTireVisual() {
        foreach(Transform tireTransform in tireTransformList) {
            RaycastHit tireRay;
            Physics.Raycast(tireTransform.position, -tireTransform.transform.up, out tireRay, springDistance);

            foreach(Transform tireTransformVisual in tireTransformVisualList) {
                if(tireTransformVisual.transform.parent == tireTransform.transform) {
                    tireTransformVisual.position = tireRay.point;
                    tireTransformVisual.position += tireTransform.up * tireRadiusVisual;

                    if(tireRay.collider == null) {
                        tireTransformVisual.position = tireTransform.position;
                        tireTransformVisual.position += -tireTransform.up * springDistance + tireTransform.up * tireRadiusVisual;
                    }
                }
            }
        }
    }

}
