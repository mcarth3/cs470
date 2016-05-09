using UnityEngine;
using System.Collections;

public class MazeNav_AStar : MonoBehaviour {

	//MazeShould have all AprilTags as directChildren
	public Transform maze;
	GameObject[] nodeArr;
	float stepSize;
	public GameObject nodePrefab;
	float nodesAcrossPerSquare = 2;

	void Start() {
		stepSize = 1.0f / nodesAcrossPerSquare;
		createGridNodes ();
		pruneGridNodes ();
		float diagonalDistance = Mathf.Sqrt (2 * stepSize * stepSize);
		populateAllConnections (diagonalDistance );
	}

	//This function returns a grid of nodes
	//  covering the spread of obstacle tags
	void createGridNodes() {
		float mazeMinX = 99999;
		float mazeMaxX = -99999;
		float mazeMinY = 99999;
		float mazeMaxY = -99999;

		foreach (Transform child in maze) {
			float nodeMinX = child.localPosition.x - 0.5f;
			float nodeMaxX = child.localPosition.x + 0.5f;
			float nodeMinY = child.localPosition.z - 0.5f;
			float nodeMaxY = child.localPosition.z + 0.5f;

			if (nodeMinX < mazeMinX) {	mazeMinX = nodeMinX;	}
			if (nodeMaxX > mazeMaxX) {	mazeMaxX = nodeMaxX;	}
			if (nodeMinY < mazeMinY) {	mazeMinY = nodeMinY;	}
			if (nodeMaxY > mazeMaxY) {	mazeMaxY = nodeMaxY;	}
		}
			
		int nodeI = 0;
		int totalNodesWide = (int)((mazeMaxX-mazeMinX+stepSize) / stepSize);
		int totalNodesTall = (int)((mazeMaxY-mazeMinY+stepSize) / stepSize);
		int totalNodes = totalNodesWide * totalNodesTall;
		nodeArr = new GameObject[totalNodes];
		for (float nodeX = mazeMinX; nodeX <= mazeMaxX; nodeX += stepSize) {
			for (float nodeY = mazeMinY; nodeY <= mazeMaxY; nodeY += stepSize) {
				nodeArr[nodeI++] = Instantiate (nodePrefab, new Vector3 (nodeX, 0.1f, nodeY), Quaternion.identity) as GameObject;
			}
		}
	}

	void pruneGridNodes() {
		foreach (Transform child in maze) {
			Rect rect = new Rect (child.position.x - 0.55f, child.position.z - 0.55f, 1.1f, 1.1f);
			foreach (GameObject node in nodeArr) {
				if (!node.gameObject.activeSelf)
					continue;

				if (rect.Contains (new Vector2( node.transform.position.x, node.transform.position.z))) {
					node.SetActive (false);
				}
			}
		}
	}

	void populateAllConnections( float neighborDistance ) {
		foreach (GameObject node in nodeArr) {
			foreach (GameObject neighbor in nodeArr) {
				if (node != neighbor && Vector3.Distance (node.transform.position, neighbor.transform.position) <= neighborDistance) {
//					node.GetComponent<Node> ().activeNeighborNodeArr [node.GetComponent<Node> ().activeNeighborNodeArr.Length] = neighbor.transform;
				}
			}
		}
	}
}
