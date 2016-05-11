using UnityEngine;
using System.Collections;

public class BasicAgent : MonoBehaviour {

	public GameObject[] goalArr;
	GameObject[] obstacleArr;

	IEnumerator Start() {
		obstacleArr = null;
		yield return new WaitForSeconds (1);
		obstacleArr = GameObject.FindGameObjectsWithTag ("BasicObstacle");

	}

	// Update is called once per frame
	void Update () {
		Vector3 velocity = Vector3.zero;
		foreach( GameObject goal in goalArr ) {
			velocity += BasicAffectorsInteractions.getAffectFromAffector (transform, goal) * 0.5f;
		}
			
		if (obstacleArr != null) {
			foreach (GameObject obstacle in obstacleArr) {
				if (Vector3.Distance (obstacle.transform.position, transform.position) <= 2.5f)
					velocity += BasicAffectorsInteractions.getAffectFromAffector (transform, obstacle) * 2;
			}
		}

//		velocity += Random.insideUnitSphere * 0.5f;

		gameObject.GetComponent<Rigidbody> ().velocity = velocity * 0.3f;
	}
}
