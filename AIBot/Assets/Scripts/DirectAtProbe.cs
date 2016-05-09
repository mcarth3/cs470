using UnityEngine;
using System.Collections;

public class DirectAtProbe : MonoBehaviour {
	public Transform probe;

	// Use this for initialization
	void Start () {
		transform.LookAt (probe);
		transform.Rotate (Vector3.up * 180);
	}
}
