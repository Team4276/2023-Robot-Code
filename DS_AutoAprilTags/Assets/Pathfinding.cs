using UnityEngine;
using UnityEngine.AI;

public class Pathfinding : MonoBehaviour
{
    public Transform target;
    private NavMeshPath path;
    private float elapsed = 0.0f;
    //Vector3 vector2 = 
    void Start()
    {
        path = new NavMeshPath();
        elapsed = 0.0f; 
    }

    void Update()
    { 
        
        // Update the way to the goal every second.
        Vector3 vector1 = transform.position;
        Vector3 vector2 = new Vector3(5.731f, 0.145f, -2.15f);
        //Vector3 test1 = new Vector3(0f,0f,0f);
        //Vector3 test2 = new Vector3(0f,12f,0f);
        elapsed += Time.deltaTime;
        int z = 0;

        if (elapsed > 1.0f)
        {
            elapsed -= 1.0f;
            NavMesh.CalculatePath(vector1,vector2 , NavMesh.AllAreas, path);
            z = path.corners.Length;
            z = z - 1;
            print (path.corners.Length);
            while (z > - 1)
                {print (path.corners[z]);
                z = z - 1;
                    }
            

        }
        for ( int i = 0; i < path.corners.Length - 1; i = i + 1)
            Debug.DrawLine(path.corners[i], path.corners[i + 1], Color.red,12.0f, false); 
            
    }
}
