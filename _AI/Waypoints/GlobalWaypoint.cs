//© 2022 by MADKEV Studio, all rights reserved
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GlobalWaypoint : MonoBehaviour
{
    #region Singleton Setup & Initialization
    public static GlobalWaypoint Instance;
    public virtual void Awake()
    {
        //Singleton
        if (Instance != null && Instance != this)
        {
            Destroy(this);
        }
        else
        {
            Instance = this;
        }
    }
    #endregion

    [Header("Groups")]
    public WaypointGroup[] groups;

    /// <summary>
    /// Returns the index of a Waypoint within its group, -1 if not in the group
    /// </summary>
    /// <param name="waypoint"></param>
    /// <param name="group"></param>
    /// <returns></returns>
    public int GetWaypointIndex(Waypoint waypoint, WaypointGroup group)
    {
        for (int i = 0; i < group.waypoints.Length; i++)
        {
            if (group.waypoints[i] == waypoint)
                return i;
        }
        return -1;
    }
    /// <summary>
    /// Returns a random Waypoint from globalWaypoint group. Uses HMSpawnPoints for levels other than Forest.
    /// </summary>
    /// <returns></returns>
    public Waypoint GetRandomWaypoint()
    {
        WaypointGroup group = GetRandomWaypointGroup();
        return group.waypoints[Random.Range(0, group.waypoints.Length)];
    }

    /// <summary>
    /// Returns current waypoint.next, if current or current.next is null, picks a random waypoint.
    /// </summary>
    /// <param name="current"></param>
    /// <returns></returns>
    public Waypoint GetNextWaypoint(Waypoint current)
    {
        if (current && current.next)
        {
            return current.next;
        }

        return GetRandomWaypoint();
    }

    public WaypointGroup GetRandomWaypointGroup()
    {
        return groups[Random.Range(0, groups.Length)];
    }

    /// <summary>
    /// Gets the closest waypoint to worldPos in a specific group. Finds a random group if no group is provided.
    /// </summary>
    /// <param name="worldPos">The world position to find the closest waypoint for.</param>
    /// <param name="group">The waypoint group to search within, or null to select a random group.</param>
    /// <returns>The closest Waypoint object to the given position.</returns>
    public Waypoint GetClosestWaypoint(Vector3 worldPos, WaypointGroup group = null)
    {
        if (group == null)
        {
            group = GetRandomWaypointGroup();
        }

        Waypoint closestWaypoint = null;
        float closestDistance = float.MaxValue;
        foreach (Waypoint waypoint in group.waypoints)
        {
            float distance = Vector3.Distance(worldPos, waypoint.transform.position);
            if (closestWaypoint == null || distance < closestDistance)
            {
                closestWaypoint = waypoint;
                closestDistance = distance;
            }
        }

        return closestWaypoint;
    }

    /// <summary>
    /// Gets the farthest waypoint to worldPos in a specific group. Finds a random group if no group is provided.
    /// </summary>
    /// <param name="worldPos">The world position to find the closest waypoint for.</param>
    /// <param name="group">The waypoint group to search within, or null to select a random group.</param>
    /// <returns>The farthest Waypoint object to the given position.</returns>
    public Waypoint GetFarthestWaypoint(Vector3 worldPos, WaypointGroup group = null)
    {
        if (group == null)
        {
            group = GetRandomWaypointGroup();
        }

        Waypoint farthestWaypoint = null;
        float farthestDistance = 0;
        foreach (Waypoint waypoint in group.waypoints)
        {
            float distance = Vector3.Distance(worldPos, waypoint.transform.position);
            if (farthestWaypoint == null || distance > farthestDistance)
            {
                farthestWaypoint = waypoint;
                farthestDistance = distance;
            }
        }

        return farthestWaypoint;
    }
}
