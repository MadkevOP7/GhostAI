//© 2022 by MADKEV Studio, all rights reserved
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEngine.SceneManagement;
[CustomEditor(typeof(Waypoint))]
public class WaypointEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        if (SceneManager.GetActiveScene().name != "Forest")
        {
            GUILayout.Label("Editor disabled as scene is not Forest");
            return;
        }
        Waypoint waypoint = (Waypoint)target;

        if (GUILayout.Button("Insert Waypoint Before"))
        {
            InsertWaypointBefore(waypoint);
        }

        if (GUILayout.Button("Insert Waypoint After"))
        {
            InsertWaypointAfter(waypoint);
        }
    }

    void InsertWaypointBefore(Waypoint w)
    {
        if (w.group == null)
        {
            w.group = w.transform.GetComponentInParent<WaypointGroup>();
        }
        GameObject temp = new GameObject(w.group._name + " waypoint");
        Waypoint _new = temp.AddComponent<Waypoint>();
        temp.transform.SetParent(w.group.transform);
        temp.transform.SetAsLastSibling();
        temp.transform.localPosition = w.transform.localPosition;

        if (w.prev == null)
        {
            w.prev = _new;
            _new.next = w;
            return;
        }
        _new.prev = w.prev;
        _new.next = w;

        _new.prev.next = _new;
        w.prev = _new;
        Selection.SetActiveObjectWithContext(temp, temp);
    }

    void InsertWaypointAfter(Waypoint w)
    {
        if (w.group == null)
        {
            w.group = w.transform.GetComponentInParent<WaypointGroup>();
        }
     
        GameObject temp = new GameObject(w.group._name + " waypoint");
        Waypoint _new = temp.AddComponent<Waypoint>();
        temp.transform.SetParent(w.group.transform);
        temp.transform.SetAsLastSibling();
        temp.transform.localPosition = w.transform.localPosition;

        if (w.next == null)
        {
            w.next = _new;
            _new.prev = w;
            return;
        }
        _new.prev = w;
        _new.next = w.next;

        w.next.prev = _new;
        w.next = _new;
        Selection.SetActiveObjectWithContext(temp, temp);
    }
}

