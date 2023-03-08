using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class w : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        GameObject parentObject = transform.parent.gameObject;
        GameObject childObject = transform.gameObject;

        Vector3 parentPosition = parentObject.transform.position;

        childObject.transform.position = parentPosition;
        Debug.Log(transform.position);

    }
}
