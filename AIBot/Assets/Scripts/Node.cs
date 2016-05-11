using UnityEngine;
using System.Collections;

public class Node : MonoBehaviour {

	public Transform[] activeNeighborNodeArr;

	// Use this for initialization
	void Start () {
		activeNeighborNodeArr = new Transform[8];
	}
	
	// Update is called once per frame
	void Update () {
		if (Input.GetMouseButtonDown (0)) {
//			if( Physics.Raycast( Camera.
		}

		if (Input.GetMouseButtonUp (0)) {

		}
	}
}
