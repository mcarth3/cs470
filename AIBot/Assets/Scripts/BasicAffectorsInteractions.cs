using UnityEngine;
using System.Collections;

public class BasicAffectorsInteractions : MonoBehaviour {

	static public Vector3 getAffectFromAffector( Transform agent, GameObject affector ) {

		string affectorType = affector.tag;

		Vector3 distance = affector.transform.position - agent.position;

		float topSpeed = 15;
		float obstacleRadius = 1.5f;

		switch (affectorType) {
		case "BasicGoal":
			if (distance.magnitude > topSpeed)
				distance = distance.normalized * topSpeed;
			return distance ;

		case "BasicObstacle":
			if (distance.magnitude < obstacleRadius)
				return distance.normalized * -10;
			break;
		}
		return Vector3.zero;
	}
}
