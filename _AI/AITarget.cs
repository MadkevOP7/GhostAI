using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class AITarget
{
    public Transform transform;

    public enum Type
    {
        player,
        wayPoint
    }

    public Type type;
}
