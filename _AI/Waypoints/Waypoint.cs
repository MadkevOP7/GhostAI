//© 2022 by MADKEV Studio, all rights reserveds
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Waypoint : MonoBehaviour
{
    public WaypointGroup group;
    public Waypoint prev;
    public Waypoint next;

    public float maxWidth = 8f;
}
