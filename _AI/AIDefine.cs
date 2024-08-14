using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AIDefine : MonoBehaviour
{
    public bool ignoreLevelCap = true;
    public enum MaxAILevelCap
    {
        beginner = 2,
        intermediate = 3,
        professional = 4,
        extreme = 5
    }
}
