 using UnityEngine;
    using UnityEngine.AI;
 public class scrip: MonoBehaviour
 {
     private NavMeshPath path;
     
     public LineRenderer line; //to hold the line Renderer
     public Transform target; //to hold the transform of the target
     public UnityEngine.AI.NavMeshAgent agent; //to hold the agent of t$$anonymous$$s gameObject
 
     public void GetPath()
     {
         line.SetPosition(0, transform.position); //set the line's origin
 
         agent.SetDestination(target.position); //create the path
 
         DrawPath(agent.path);
 
         //agent.Stop();//add t$$anonymous$$s if you don't want to move the agent
     }
 
     public void DrawPath(UnityEngine.AI.NavMeshPath path)
     {
         if(path.corners.Length < 2) //if the path has 1 or no corners, there is no need
             return;
 
         line.SetVertexCount(path.corners.Length); //set the array of positions to the amount of corners
 
         for(int i = 1; i < path.corners.Length; i++){
             line.SetPosition(i, path.corners[i]); //go through each corner and set that to the line renderer's position
         }
 }
 void update(){
path = new NavMeshPath();
GetPath();
DrawPath(path);

 }
 }