//© 2022 by MADKEV Studio, all rights reserved
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WaypointGroup : MonoBehaviour
{
    [Header("Settings")]
    public string _name = "New Waypoint Group";
    public Color color = Color.blue;

    public Waypoint[] waypoints;

    private void OnDrawGizmos()
    {
        foreach(Transform t in transform)
        {
            Gizmos.color = color;
            Gizmos.DrawSphere(t.position, 1);
            if (t.GetComponent<Waypoint>().next != null)
            {
                Gizmos.DrawLine(t.transform.position, t.GetComponent<Waypoint>().next.transform.position);
            }
        }
    }
}
