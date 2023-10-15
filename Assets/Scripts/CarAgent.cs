using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarAgent : MonoBehaviour {


    private List<Vector3> roadPath = new List<Vector3> {
        new Vector3(0,0,0),
        new Vector3(0,0,1),
        new Vector3(1,0,2),
        new Vector3(2,0,2),
        new Vector3(3,0,2),
        new Vector3(4,0,3),
        new Vector3(4,0,4),
        new Vector3(3,0,4),
        new Vector3(3,0,3),
        new Vector3(2,0,2),
    };
    private int currentWayPoint;

    CarController carController;
    Rigidbody body;

    private void Awake() {
        carController = GetComponent<CarController>();
        body = GetComponent<Rigidbody>();
    }


    private void Update() {
        carController.Agent_Steer(Vector3.SignedAngle(transform.forward, nextWayPoint() - transform.position, Vector3.up));
    }



    private Vector3 nextWayPoint() {
        if (roadPath.Count == 0) return transform.forward;

        currentWayPoint = Mathf.Min(currentWayPoint, roadPath.Count - 1);

        if ((roadPath[currentWayPoint] - transform.position).magnitude < 1) {
            Debug.Log("got here");
            if (roadPath.Count - 1 > currentWayPoint) currentWayPoint++;
            else Debug.Log("finish");
        }

        return roadPath[Mathf.Min(currentWayPoint, roadPath.Count - 1)];
    }







    [ExecuteInEditMode]
    private void OnDrawGizmos() {
        if (body == null) return;
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(nextWayPoint(), 0.3f);
    }
}
