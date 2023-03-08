using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using FRC.NetworkTables;
using FRC.NativeLibraryUtilities;
using FRC.ILGeneration;
using System;

public class NavMeshagentCornors : MonoBehaviour
{
    public Transform destination;
    public Color pathColor = Color.red;
    public NetworkTable nt;

    public string tableName = "PathCorners";
    public string entryName = "Corners";

    private NavMeshAgent agent;
    private LineRenderer lineRenderer;

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        lineRenderer = gameObject.AddComponent<LineRenderer>();

        // get the path to the destination
        NavMeshPath path = new NavMeshPath();
        agent.CalculatePath(destination.position, path);

        // send the path corners to NetworkTables
        Vector3[] pathCorners = path.corners;
        List<double> cornerValues = new List<double>();
        
        // seting up the line render in the middle here in case no connection is made to roborio
        lineRenderer.startWidth = 0.3f;
        lineRenderer.endWidth = 0.3f;
        lineRenderer.material.color = pathColor;
        lineRenderer.positionCount = pathCorners.Length;
        lineRenderer.SetPositions(pathCorners);
        Debug.Log(lineRenderer.isVisible);


        for (int i = 0; i < pathCorners.Length; i++)
        {
            cornerValues.Add(Math.Round(pathCorners[i].x, 4));
            cornerValues.Add(Math.Round(pathCorners[i].y, 4));
            cornerValues.Add(Math.Round(pathCorners[i].z, 4));
        }
        
        Debug.Log(cornerValues[3]);
        Debug.Log(pathCorners[1]);

        // Send the corner values to NetworkTables
        NetworkTableEntry entry = nt.GetEntry(tableName + "/" + entryName);
        entry.SetDoubleArray(cornerValues.ToArray());

    }


    void Update()
    {
        if (agent.remainingDistance < agent.stoppingDistance)
        {
            // Path complete
            Debug.Log("Path completed");
        }
    }
}
