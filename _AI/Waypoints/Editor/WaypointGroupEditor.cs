//© 2022 by MADKEV Studio, all rights reserved
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(WaypointGroup))]
public class WaypointGroupEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        WaypointGroup selection = (WaypointGroup)target;

        if (GUILayout.Button("Register Waypoint Group"))
        {
            RegisterWaypointGroup(selection);
        }
    }

    void RegisterWaypointGroup(WaypointGroup g)
    {
        g.waypoints = g.GetComponentsInChildren<Waypoint>();
        g.transform.name = g._name;
        for (int i = 0; i < g.waypoints.Length; i++)
        {
            g.waypoints[i].gameObject.name = g._name + " waypoint " + i;
            g.waypoints[i].group = g;
            g.waypoints[i].transform.position += new Vector3(0, 1000, 0);
            RaycastHit hit;
            if (Physics.Raycast(g.waypoints[i].transform.position, -g.waypoints[i].transform.up, out hit, Mathf.Infinity))
            {
                g.waypoints[i].transform.position = hit.point;
            }
        }
    }
}
