﻿using UnityEngine;
using System.Collections;

public class PotentialLookAt : MonoBehaviour {
	public Transform target;

	void Update() {
		// Rotate the camera every frame so it keeps looking at the target 
		transform.LookAt(target);
	}
}