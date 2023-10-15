// REFERENCE https://youtu.be/CdPYlj5uZeI or https://www.youtube.com/watch?v=CdPYlj5uZeI&t=13s&ab_channel=ToyfulGames

using System.Collections.Generic;
using UnityEngine;

public class CarController : MonoBehaviour {

    [SerializeField] private List<Transform> tireTransformList;

    [SerializeField] private List<Transform> tireToSteerTransformList;
    [SerializeField] private List<Transform> tireToApplyAccelerationForceTransformList;

    [SerializeField] private List<Transform> tireTransformVisualList;
    private Rigidbody body;

    [SerializeField] private float springStrength = 300f;
    [SerializeField] private float springDamper = 5f;
    [SerializeField] private float springDistance = 0.4f;

    [SerializeField] private float tireStaticFriction;
    [SerializeField] private float tireKinematicFriction;

    [SerializeField] private AnimationCurve powerCurve;
    [SerializeField] private float carTopSpeed = 20f;
    [SerializeField] private float acceleration = 100f;
    [SerializeField] private float deceleration = 80f;
    [SerializeField] private float autoDecelerationForce = 20f;

    private float tireRadiusVisual = 0.25f;

    [SerializeField] private AnimationCurve steerCurve;



    private float steerInputNormalized = 0f;
    private float throttleInputNormalized = 0f;

    private void Awake() {
        body = GetComponent<Rigidbody>();
    }

    private void Update() {
        HandleSteering();
        HandleTireVisual();


        if (Input.GetKey(KeyCode.E)) {
            transform.rotation = Quaternion.identity;
            transform.position += Vector3.up * 0.1f;
            body.velocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
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

        if (Input.GetKey(KeyCode.A)) {
            steerInputNormalized = Mathf.Lerp(steerInputNormalized, -1, steerSpeed);
        }
        if (Input.GetKey(KeyCode.D)) {
            steerInputNormalized = Mathf.Lerp(steerInputNormalized, 1, steerSpeed);
        }
        if (!Input.GetKey(KeyCode.A) && !Input.GetKey(KeyCode.D)) {
            steerInputNormalized = Mathf.Lerp(steerInputNormalized, 0, steerSpeed / 2);
        }



        float carSpeed = Vector3.Dot(transform.forward, body.velocity);
        float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);
        float steeringAfterSpeedCorrection = steerCurve.Evaluate(normalizedSpeed) * steerInputNormalized * maxSteeringAngle;

        Vector3 slideDirection = gameObject.transform.right;
        Vector3 carWorldVelocity = body.velocity;
        float sideVelocity = Vector3.Dot(slideDirection, carWorldVelocity);

        float steeringCorrection = 1.1f;
        steeringAfterSpeedCorrection += sideVelocity * steeringCorrection;

        foreach (Transform tireTransform in tireToSteerTransformList) {
            tireTransform.localEulerAngles = new Vector3(0, steeringAfterSpeedCorrection, 0);
        }
    }

    private void HandleAccelerationAndDeceleration() {
        float throttleChangeSpeed = 10f;

        if (Input.GetKey(KeyCode.W)) {
            throttleInputNormalized = Mathf.Lerp(throttleInputNormalized, 1, throttleChangeSpeed);
        }
        if (Input.GetKey(KeyCode.S)) {
            throttleInputNormalized = Mathf.Lerp(throttleInputNormalized, -1, throttleChangeSpeed);
        }
        if (!Input.GetKey(KeyCode.W) && !Input.GetKey(KeyCode.S)) {
            throttleInputNormalized = Mathf.Lerp(throttleInputNormalized, 0, throttleChangeSpeed);
        }

        if (Input.GetKey(KeyCode.LeftShift)) {
            throttleInputNormalized *= 1.5f;
        }

        foreach (Transform tireTransform in tireToApplyAccelerationForceTransformList) {
            if (Physics.Raycast(tireTransform.position, -tireTransform.transform.up, out RaycastHit tireRay, springDistance)) {
                Vector3 tireDirection = tireTransform.forward;
                Vector3 tireWorldVelocity = body.GetPointVelocity(tireTransform.position);
                Vector3 forwardByNormal = Vector3.Dot(tireDirection, Vector3.Cross(tireTransform.right, tireRay.normal)) * Vector3.Cross(tireTransform.right, tireRay.normal);

                //powerCurve.Evaluate(normalizedSpeed)

                float forceToApply = 0;
                float directionOfForce = 0;

                // auto decelerating
                if (throttleInputNormalized == 0) {
                    forceToApply = autoDecelerationForce;
                    if (forwardByNormal.magnitude > 0.1f)
                        directionOfForce = Vector3.Dot(tireDirection, tireWorldVelocity) > 0 ? -1 : 1;
                }
                else {
                    // acceletating or decelerating
                    forceToApply = Vector3.Dot(tireDirection * throttleInputNormalized, tireWorldVelocity) > 0 ? acceleration : deceleration;
                    directionOfForce = throttleInputNormalized;
                }

                //Debug.Log("force: " + forceToApply * directionOfForce);
                body.AddForceAtPosition(forwardByNormal.normalized * directionOfForce * forceToApply / tireToApplyAccelerationForceTransformList.Count * Time.fixedDeltaTime, tireRay.point);
            }
        }
    }

    private void HandleSideGrip() {
        foreach (Transform tireTransform in tireTransformList) {
            if (Physics.Raycast(tireTransform.position, -tireTransform.transform.up, out RaycastHit tireRay, springDistance)) {
                Vector3 velocityRightComponent = Vector3.Dot(body.GetPointVelocity(tireTransform.position), Vector3.Cross(tireTransform.forward, tireRay.normal)) * Vector3.Cross(tireTransform.forward, tireRay.normal);
                Vector3 velocityForwardComponent = Vector3.Dot(body.GetPointVelocity(tireTransform.position), Vector3.Cross(tireTransform.right, tireRay.normal)) * Vector3.Cross(tireTransform.right, tireRay.normal);

                Vector3 sideForce = velocityRightComponent.normalized * body.mass / tireTransformList.Count * (springDistance - tireRay.distance);


                body.AddForceAtPosition(-sideForce * tireKinematicFriction * Time.fixedDeltaTime, tireRay.point);
                //if (sideForce.magnitude > 1)
                //    body.AddForceAtPosition(-sideForce * tireKinematicFriction * Time.fixedDeltaTime, tireRay.point);
                //else
                //    body.AddForceAtPosition(-sideForce * tireStaticFriction * Time.fixedDeltaTime, tireRay.point);


                Debug.DrawRay(tireRay.point + tireTransform.up, sideForce * 100, Color.red);
                Debug.DrawRay(tireRay.point + tireTransform.up, Vector3.Cross(tireTransform.right, tireRay.normal) * 100, Color.blue);
            }
        }
    }

    private void HandleSuspention() {
        foreach (Transform tireTransform in tireTransformList) {
            Debug.DrawRay(tireTransform.position, -tireTransform.transform.up * springDistance, Color.black);

            if (Physics.Raycast(tireTransform.position, -tireTransform.transform.up, out RaycastHit tireRay, springDistance)) {
                Vector3 springDirection = tireTransform.up;
                Vector3 tireWorldVelocity = body.GetPointVelocity(tireTransform.position);

                float offset = springDistance - tireRay.distance;
                float springStrengthPerWheel = springStrength / tireTransformList.Count;
                float springForce = (offset * springStrengthPerWheel);

                float velocity = Vector3.Dot(springDirection, tireWorldVelocity);
                float damperForce = -Mathf.Min(velocity * springDamper, springForce);

                body.AddForceAtPosition(springDirection * (springForce + damperForce), tireRay.point);
            }
        }
    }

    private void HandleTireVisual() {
        foreach (Transform tireTransform in tireTransformList) {
            RaycastHit tireRay;
            Physics.Raycast(tireTransform.position, -tireTransform.transform.up, out tireRay, springDistance);

            foreach (Transform tireTransformVisual in tireTransformVisualList) {
                if (tireTransformVisual.transform.parent == tireTransform.transform) {
                    tireTransformVisual.position = tireRay.point;
                    tireTransformVisual.position += tireTransform.up * tireRadiusVisual;

                    if (tireRay.collider == null) {
                        tireTransformVisual.position = tireTransform.position;
                        tireTransformVisual.position += -tireTransform.up * springDistance + tireTransform.up * tireRadiusVisual;
                    }
                }
            }
        }
    }

}
