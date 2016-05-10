using UnityEngine;
using System.Collections;

public class AffectorMover : MonoBehaviour {

	public GameObject[] affectorArr;

	// Use this for initialization
	void Start () {
		iTween.MoveBy (affectorArr[1], iTween.Hash ("x", 1, "time", 2, "looptype", iTween.LoopType.pingPong, "easetype", iTween.EaseType.linear ));
		iTween.RotateBy( affectorArr[2], iTween.Hash( "z", 1, "time", 7, "looptype", iTween.LoopType.loop, "easetype", iTween.EaseType.linear ));
		iTween.RotateBy( affectorArr[3], iTween.Hash( "y", -1, "time", 14, "looptype", iTween.LoopType.loop, "easetype", iTween.EaseType.linear ));
	}
}
