using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AIMath : MonoBehaviour
{
    /// <summary>
    /// Makes a randomized decision, given a percentage of picking the left option (return true). Right returns false.
    /// </summary>
    /// <param name="probabilityPercentage"></param>
    /// <returns></returns>
    public static bool Decide2(int probabilityPercentage)
    {
        int r = Random.Range(1, 101);
        return (r <= probabilityPercentage);
    }

    /// <summary>
    /// Makes a randomized decision, ie random boolean
    /// </summary>
    /// <returns></returns>
    public static bool Decide2()
    {
        int r = Random.Range(1, 101);
        return r <= 50;
    }
}
