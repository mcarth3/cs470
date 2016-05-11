using UnityEngine;
using System.Collections;

public class AsteroidGenerator : MonoBehaviour {

	public GameObject obstaclePrefab;
	public GameObject[] asteroidMeshArr;
	public int total = 100;
	public int radius = 10;

	// Use this for initialization
	void Start () {
		createAsteroids (total);
		if( asteroidsShouldMove )
			iTween.RotateBy (gameObject, iTween.Hash ("y", 1, "time", 30, "looptype", iTween.LoopType.loop, "easetype", iTween.EaseType.linear));
	}
		
	public Material trailMat;
	public bool asteroidsShouldMove = true;
	public bool showAsteroidTrails = false;
	void createAsteroids( int totalAsteroids ) {
		for (int i = 0; i < totalAsteroids; i++) {
			GameObject obstacle = Instantiate (obstaclePrefab, Random.insideUnitSphere * radius, Quaternion.identity) as GameObject;
			obstacle.GetComponent<Renderer> ().enabled = false;

			GameObject asteroid = Instantiate( asteroidMeshArr[Random.Range(0, asteroidMeshArr.Length)], Vector3.zero, Random.rotation ) as GameObject;
			asteroid.transform.parent = obstacle.transform;
			asteroid.transform.localPosition = Vector3.zero;
			asteroid.transform.localScale = Vector3.one * 100;

			iTween.RotateBy (asteroid, iTween.Hash ("amount", Random.onUnitSphere, "time", 25, "looptype", iTween.LoopType.loop, "easetype", iTween.EaseType.linear));
			if (showAsteroidTrails) {
				asteroid.AddComponent<TrailRenderer> ();
				asteroid.GetComponent<TrailRenderer> ().material = trailMat;
				asteroid.GetComponent<TrailRenderer> ().startWidth = 0.2f;
				asteroid.GetComponent<TrailRenderer> ().endWidth = 0.2f;
				asteroid.GetComponent<TrailRenderer> ().time = 100;
			}
			obstacle.transform.parent = transform;
		}
	}
}
