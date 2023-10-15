// REFERENCE https://youtu.be/CdPYlj5uZeI or https://www.youtube.com/watch?v=CdPYlj5uZeI&t=13s&ab_channel=ToyfulGames

using System.Collections.Generic;
using System.IO.Pipes;
using UnityEngine;

public class CarController : MonoBehaviour {

    [SerializeField] private List<Transform> tireTransformList;

    [SerializeField] private List<Transform> tireToSteerTransformList;
    [SerializeField] private List<Transform> tireToApplyAccelerationForceTransformList;

    [SerializeField] private List<Transform> tireTransformVisualList;

    [Header("Suspention")]
    [SerializeField, Min(0)] private float springStrength = 300f;
    [SerializeField, Min(0)] private float springDamper = 5f;
    [SerializeField, Min(0)] private float springDistance = 0.4f;

    [Header("Wheels")]
    [SerializeField, Min(0)] private float applyKinematicAfterVelocity = 0.3f;
    [SerializeField, Min(0)] private float tireStaticFriction = 1000f;
    [SerializeField, Min(0)] private float tireKinematicFriction = 500f;
    [SerializeField, Min(0)] private float tireRadiusVisual = 0.25f;
    [SerializeField, Min(0)] private float tireWidth = 0.1f;

    [Header("Engine")]
    [SerializeField] private AnimationCurve powerCurve;
    [SerializeField, Min(0)] private float carTopSpeed = 20f;
    [SerializeField, Min(0)] private float acceleration = 100f;
    [SerializeField, Min(0)] private float deceleration = 80f;
    [SerializeField, Min(0)] private float autoDecelerationForce = 20f;

    [Header("Steering")]
    [SerializeField] private AnimationCurve steerCurve;
    [SerializeField, Min(0)] private float maxSteeringAngle = 45f;
    [SerializeField, Min(0)] private float steerSpeed = 20f;
    private float currentWheelAngle = 0;

    private Rigidbody body;


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
        float steerInputNormalized = 0f;

        if (Input.GetKey(KeyCode.A)) {
            steerInputNormalized--;
        }
        if (Input.GetKey(KeyCode.D)) {
            steerInputNormalized++;
        }
        // straightning
        if (steerInputNormalized == 0) {
            steerInputNormalized = Mathf.Lerp(steerInputNormalized, 0, steerSpeed / 2);
        }


        float carSpeed = Vector3.Dot(transform.forward, body.velocity);
        float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);
        float desiredWheelAngle = steerCurve.Evaluate(normalizedSpeed) * steerInputNormalized * maxSteeringAngle;


        //// correction
        //float steeringCorrection = 0.9f;
        //float sideVelocity = Vector3.Dot(transform.right, body.velocity);
        //wheelsAngle += sideVelocity * steeringCorrection;

        foreach (Transform tireTransform in tireToSteerTransformList) {
            currentWheelAngle = Mathf.Lerp(currentWheelAngle, desiredWheelAngle, steerSpeed * Time.deltaTime);
            tireTransform.localEulerAngles = new Vector3(0, currentWheelAngle, 0);
        }
    }

    private void HandleAccelerationAndDeceleration() {
        float throttleInputNormalized = 0f;

        if (Input.GetKey(KeyCode.W)) {
            throttleInputNormalized++;
        }
        if (Input.GetKey(KeyCode.S)) {
            throttleInputNormalized--;
        }
        if (Input.GetKey(KeyCode.LeftShift)) {
            throttleInputNormalized *= 1.5f;
        }

        float forceToApply = 0;
        float directionOfForce = 0;
        // auto decelerating
        if (throttleInputNormalized == 0) {
            forceToApply = autoDecelerationForce;
            if (body.velocity.magnitude > 0.1f) directionOfForce = Vector3.Dot(transform.forward, body.velocity) > 0 ? -1 : 1;
        }
        else {
            // acceletating or decelerating
            float throtleToApply = acceleration * powerCurve.Evaluate(Mathf.Clamp01(Mathf.Abs(Vector3.Dot(transform.forward, body.velocity)) / carTopSpeed));

            forceToApply = Vector3.Dot(transform.forward * throttleInputNormalized, body.velocity) > 0 ? throtleToApply : deceleration;
            directionOfForce = throttleInputNormalized;
        }

        foreach (Transform tireTransform in tireToApplyAccelerationForceTransformList) {
            if (Physics.Raycast(tireTransform.position, -tireTransform.transform.up, out RaycastHit tireRay, springDistance)) {
                Vector3 tireDirection = tireTransform.forward;
                Vector3 forwardByNormal = Vector3.Dot(tireDirection, Vector3.Cross(tireTransform.right, tireRay.normal).normalized) * Vector3.Cross(tireTransform.right, tireRay.normal).normalized;

                /*
                Vector3 tireWorldVelocity = body.GetPointVelocity(tireTransform.position);

                if (throttleInputNormalized == 0) {
                    forceToApply = autoDecelerationForce;
                    if (forwardByNormal.magnitude > 0.1f) directionOfForce = Vector3.Dot(tireDirection, tireWorldVelocity) > 0 ? -1 : 1;
                }
                else {
                    // acceletating or decelerating
                    float throtleToApply = acceleration * powerCurve.Evaluate(Mathf.Clamp01(Mathf.Abs(Vector3.Dot(transform.forward, body.velocity)) / carTopSpeed));
                    Debug.Log("throtleToApply: " + throtleToApply);
                
                    forceToApply = Vector3.Dot(tireDirection * throttleInputNormalized, tireWorldVelocity) > 0 ? throtleToApply : deceleration;
                    directionOfForce = throttleInputNormalized;
                }
                 */

                body.AddForceAtPosition(forwardByNormal * directionOfForce * forceToApply / tireToApplyAccelerationForceTransformList.Count * Time.fixedDeltaTime, tireRay.point);
            }
        }
    }

    private void HandleSideGrip() {
        foreach (Transform tireTransform in tireTransformList) {
            if (Physics.Raycast(tireTransform.position, -tireTransform.transform.up, out RaycastHit tireRay, springDistance)) {
                Vector3 velocityRightComponent = Vector3.Dot(body.GetPointVelocity(tireTransform.position), Vector3.Cross(tireTransform.forward, tireRay.normal).normalized) * Vector3.Cross(tireTransform.forward, tireRay.normal).normalized;

                Vector3 sideVelocity = velocityRightComponent * body.mass / tireTransformList.Count * (springDistance - tireRay.distance);

                if (sideVelocity.magnitude < applyKinematicAfterVelocity) {
                    body.AddForceAtPosition(-sideVelocity * tireStaticFriction * Time.fixedDeltaTime, tireRay.point);
                }
                else
                    body.AddForceAtPosition(-sideVelocity.normalized * tireKinematicFriction * Time.fixedDeltaTime, tireRay.point);


                //Debug.Log("sideVelocity.magnitude: " + sideVelocity.magnitude);
                Debug.DrawRay(tireRay.point + tireTransform.up, sideVelocity * 10, Color.red);
                Debug.DrawRay(tireRay.point + tireTransform.up, Vector3.Cross(tireTransform.right, tireRay.normal) * 10, Color.blue);
            }
        }
    }

    private void HandleSuspention() {
        foreach (Transform tireTransform in tireTransformList) {
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
        foreach (Transform tireTransformVisual in tireTransformVisualList) {
            Transform parentTire = tireTransformVisual.transform.parent;
            Physics.Raycast(tireTransformVisual.transform.parent.position, -parentTire.transform.up, out RaycastHit tireRay, springDistance);


            tireTransformVisual.position = parentTire.position - parentTire.up * tireRay.distance + parentTire.up * tireRadiusVisual / 2;

            if (tireRay.collider == null) {
                tireTransformVisual.position = parentTire.position - parentTire.up * springDistance + parentTire.up * tireRadiusVisual / 2;
            }

            tireTransformVisual.localScale = new Vector3(tireRadiusVisual, tireWidth, tireRadiusVisual);

            if (tireRay.collider != null)
                Debug.DrawRay(parentTire.position, parentTire.up * (springDistance - tireRay.distance), Color.black);
        }
    }




    [ExecuteInEditMode]
    private void OnDrawGizmos() {
        HandleTireVisual();
    }

}
