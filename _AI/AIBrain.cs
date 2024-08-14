//© 2022 by MADKEV Studio, all rights reserved
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mirror;
using UnityEngine.AI;
using System.Linq;

public class AIBrain : NetworkBehaviour
{
    bool PRINT_DEBUG = false;
    [Header("Runtime")]
    HostMachineManager hostmachineManager;
    public GameObject[] allGhosts;
    public List<GhostAI> spawnedGhosts = new List<GhostAI>(); //spawned ghosts
    bool hasInitialAttack; //If any machine is interacted for the first time, trigger gate failure protocol

    #region Core

    #region Callbacks/Signalling
    [Server]
    public void OnHostMachineAttacked(HostMachine hm, Player instigator = null)
    {
        //Here we are on the server
        if (!hasInitialAttack)
        {
            hasInitialAttack = true;
            GameManager.instance.ServerTriggerLimenBreak();
        }
        StartCoroutine(IEDelayedAIBrainDecide(Random.Range(1, 21)));
    }

    public void OnHostMachineDestroyed(HostMachine hm, Player instigator = null)
    {
        StartCoroutine(IEDelayedAIBrainDecide(Random.Range(1, 21)));
    }

    //We need to delay AIBrainDecide to wait for some changes (ie. Player damaging a machine, need to wait so that lowest health calculation works)
    IEnumerator IEDelayedAIBrainDecide(int delayAmount)
    {
        yield return new WaitForSeconds(delayAmount);
        OnAIBrainDecide();
    }
    private void OnInitialize()
    {
#if UNITY_EDITOR
        PRINT_DEBUG = true;
#endif
        //Initialize constants (multiplier, prob)
        LIMENDefine.Difficulty d;
        d = GameManager.instance._currentDifficulty;

        //Spawn AI
        int spawnCount = LIMENDefine.GetAIDifficultySpawnCount(d);
        PRINTDB("AIBrain: Spawning AI of count " + spawnCount + " Difficulty: " + d.ToString());

        //Todo: When we have more AI, make sure each AI spawned is unique (ie. no repeated same type)
        for (int i = 0; i < spawnCount; i++)
        {
            SpawnGhostNearPositionAuto(Random.Range(0, allGhosts.Length), hostmachineManager.GetRandomHostMachine().transform.position);
        }

        StartCoroutine(IETickAIBrain());
    }


    IEnumerator IETickAIBrain()
    {
        yield return new WaitForSeconds(120);
        OnAIBrainDecide();
        while (true)
        {
            yield return new WaitForSeconds(600); //Wait 10 minutes
            OnAIBrainDecide();
        }
    }

    private void OnAIBrainDecide()
    {

        if (AIMath.Decide2(LIMENDefine.GetAIProbabilityPercent(GameManager.instance._currentDifficulty)))
        {
            PRINTDB("AIBrain: Picked option waypoint to player");

            //Set AI Waypoint to player

            //Gets a list of free AI (not currently chasing player)
            List<GhostAI> freeAI = new List<GhostAI>();
            foreach (GhostAI ai in spawnedGhosts)
            {
                if (!ai.GetIsChasingPlayer())
                {
                    freeAI.Add(ai);
                }
            }

            //No freeAI, return
            if (freeAI.Count == 0) return;

            //Gets a list of free players (not currently being chased and alive)
            List<Player> freePlayers = new List<Player>();
            foreach (GameObject p in GameObject.FindGameObjectsWithTag("Player"))
            {
                Player pl = p.GetComponent<Player>();
                if (pl._currentChaser == null && pl.is_alive)
                {
                    freePlayers.Add(pl);
                }
            }

            //No free players, return
            if (freePlayers.Count == 0) return;

            //Sort players by health low to high
            Player[] sortedPlayerArray = freePlayers.OrderBy(x => x.health).ToArray();
            for (int i = 0; i < sortedPlayerArray.Length && i < freeAI.Count; i++)
            {
                //Assign player as way point target to AI
                freeAI[i].SetPlayerWaypointTarget(sortedPlayerArray[i]);

                PRINTDB("AIBrain: Player " + sortedPlayerArray[i].gameObject.name + " with health " + sortedPlayerArray[i].health + " is set as waypoint target to " + freeAI[i].gameObject.name);

            }
        }
        else
        {
            PRINTDB("AIBrain: Picked option waypoint to HM");

            //Waypoint to machine
            //Note: Current player chase directed by AIBrain is being set as Waypoint type, thus this would override it and make AI move to machines
            //I think it's good to override as we it seems more natural for AI to move on to machines after chasing player for some amount
            //Since AI should automatically transition to chase player target when player is in vision

            if (AIMath.Decide2(LIMENDefine.GetAIProbabilityPercent(GameManager.instance._currentDifficulty)))
            {
                PRINTDB("AIBrain: Picked option waypoint to HM with lowest health");

                //Picks HM with lowest health
                //Balance Note: this would mean that AI would likely navigate to Host Machine where player is currently at
                //Which might be too difficult for players so we apply AiProbability percent to decide

                //Gets a list of free AI (not currently chasing player)
                List<GhostAI> freeAI = new List<GhostAI>();
                foreach (GhostAI ai in spawnedGhosts)
                {
                    if (!ai.GetIsChasingPlayer())
                    {
                        freeAI.Add(ai);
                    }
                }

                //No free AI, return
                if (freeAI.Count == 0) return;

                //Gets a list of all HM machines currently alive
                List<HostMachine> aliveHMs = hostmachineManager.GetAliveHostMachines();

                //No alive host machines, return
                if (aliveHMs.Count == 0) return;

                //Sort Host Machines by ascending health
                HostMachine[] sortedHMArray = aliveHMs.OrderBy(x => x.machine_Health).ToArray();

                //Assign each host machine as way point target to AI
                for (int i = 0; i < sortedHMArray.Length && i < freeAI.Count; i++)
                {
                    freeAI[i].SetHMWaypointTarget(sortedHMArray[i]);

                    PRINTDB("AIBrain: Host Machine " + sortedHMArray[i].gameObject.name + " with health " + sortedHMArray[i].machine_Health + " is set as waypoint target to " + freeAI[i].gameObject.name);
                }
            }

        }
    }
    #endregion

    #region Internals
    /// <summary>
    /// Validates if the given ghost satisfies the options, returns true for satisfying, false otherwise
    /// </summary>
    /// <param name="ghost"></param>
    /// <param name="options"></param>
    /// <returns></returns>
    protected bool InternalValidateGhost(GhostAI ghost, ref GetGhostOptions options)
    {
        if (options.filterGhostsWithoutPlayerTarget && ghost.GetHasPlayerTarget()) return false;
        if (options.filterGhostsWithPlayerTarget && !ghost.GetHasPlayerTarget()) return false;

        //Other state validation that fails here, all not fail = true
        return true;
    }
    #endregion
    #endregion

    #region Singleton Setup & Initialization
    public static AIBrain Instance { get; private set; }
    private void Awake()
    {
        //Singleton
        if (Instance != null && Instance != this)
        {
            Destroy(Instance);
        }

        Instance = this;
    }
    #endregion

    #region Client Side Registering
    public override void OnStartClient()
    {
        base.OnStartClient();
        allGhosts = GlobalContainer.GetInstance().globalGhosts;

        //Note: Moved to centralize registering in GlobalContainer
        //foreach (GameObject g in allGhosts)
        //{
        //    NetworkClient.RegisterPrefab(g);
        //}
    }
    #endregion
    // Start is called before the first frame update
    public override void OnStartServer()
    {
        base.OnStartServer();
        hostmachineManager = GetComponent<HostMachineManager>();
        StartCoroutine(IEDelayedOnInitialize());
    }

    //Delay initialize by one frame
    IEnumerator IEDelayedOnInitialize()
    {
        yield return null;
        OnInitialize();
    }

    #region Ghost Spawning Functions (Server)
    /// <summary>
    /// [Server Only] Spawns ghost with specified index in pool at given Vector3 position, without NavMesh path sampling to ensure spawn position is valid
    /// </summary>
    /// <param name="index">The index of the ghost to spawn</param>
    /// <param name="position">The position to spawn at</param>
    [Server]
    public void SpawnGhost(int index, Vector3 position)
    {
        Vector3 pos = position + new Vector3(0, 0.1f, 0);
        GameObject g = Instantiate(allGhosts[index], pos, allGhosts[index].transform.rotation);
        NetworkServer.Spawn(g);
        spawnedGhosts.Add(g.GetComponent<GhostAI>());

#if UNITY_EDITOR
        Debug.Log("AI spawned at index: " + GetGhostByIndex(g.GetComponent<GhostAI>()));
#endif
    }

    /// <summary>
    /// [Server Only] Spawns AI given index and near provided position based on NavMesh path sampling, which validates the position to be NavMesh walkable
    /// </summary>
    /// <param name="index"></param>
    /// <param name="position"></param>
    /// <returns>True if successfully found valid point near given position, false otherwise</returns>
    [Server]
    public bool SpawnGhostNearPosition(int index, Vector3 position)
    {
        Transform nearestHMPt = FindNearestHMSpawnPointTransform(position);
        Debug.Log("Picked PT: " + nearestHMPt + " Target: " + position + " Distance: " + Vector3.Distance(position, nearestHMPt.position));
        //Calculate path from position to HMSpawnPoint nearest to position
        NavMeshPath path = new NavMeshPath();
        if (NavMesh.CalculatePath(position, nearestHMPt.position, NavMesh.AllAreas, path) && path.status != NavMeshPathStatus.PathInvalid)
        {
            //We use the last corner in the calculated path as spawn location
            //Since it should be the closest Navmesh validated position to target
            if (path.corners == null || path.corners.Length == 0)
            {
                Debug.Log("Path doesn't contain corners, invalid path!");
                return false;
            }
            SpawnGhost(index, path.corners[path.corners.Length - 1]);
            Debug.Log("Successfully spawned ghost near position: " + position + " at: " + path.corners[path.corners.Length - 1]);
            return true;
        }

        Debug.Log("Failed to spawn ghost near position: " + position);
        return false;
    }

    /// <summary>
    /// Tries to spawn ghost near position, if fails spawn at position
    /// </summary>
    /// <param name="index"></param>
    /// <param name="position"></param>
    [Server]
    public void SpawnGhostNearPositionAuto(int index, Vector3 position)
    {
        if (!SpawnGhostNearPosition(index, position))
        {
            SpawnGhost(index, position);
        }
    }
    #endregion

    #region Ghost Filtering Functions (Server)
    public enum GetGhostLevelFilterMode
    {
        FilterHigher,
        FilterHigherInclusive,
        FilterLower,
        FilterLowerInclusive
    }

    public int GetGhostByIndex(GhostAI ai)
    {
        return spawnedGhosts.IndexOf(ai);
    }

    //Returns a list of index of ghosts by certain level
    public List<int> GetGhostsByLevel(int level)
    {
        List<int> l = new List<int>();
        for (int i = 0; i < spawnedGhosts.Count; i++)
        {
            if (spawnedGhosts[i].GetIdentity().level == level)
            {
                l.Add(i);
            }
        }
        return l;
    }

    /// <summary>
    /// Gets ghosts by level with filter mode specified
    /// </summary>
    /// <param name="level"></param>
    /// <param name="mode"></param>
    /// <returns></returns>
    public List<int> GetGhostsByLevel(int level, GetGhostLevelFilterMode mode)
    {
        List<int> filteredIndices = new List<int>();
        System.Func<int, bool> filterCondition;

        switch (mode)
        {
            case GetGhostLevelFilterMode.FilterHigher:
                filterCondition = ghostLevel => ghostLevel > level;
                break;
            case GetGhostLevelFilterMode.FilterHigherInclusive:
                filterCondition = ghostLevel => ghostLevel >= level;
                break;
            case GetGhostLevelFilterMode.FilterLower:
                filterCondition = ghostLevel => ghostLevel < level;
                break;
            case GetGhostLevelFilterMode.FilterLowerInclusive:
                filterCondition = ghostLevel => ghostLevel <= level;
                break;
            default:
                throw new System.ArgumentOutOfRangeException(nameof(mode), "Unsupported filter mode");
        }

        for (int i = 0; i < spawnedGhosts.Count; i++)
        {
            var ghostLevel = spawnedGhosts[i].GetIdentity().level;
            if (filterCondition(ghostLevel))
            {
                filteredIndices.Add(i);
            }
        }

        return filteredIndices;
    }

    public GhostAI GetGhostByLevel(int level)
    {
        foreach (GhostAI ghost in spawnedGhosts)
        {
            if (ghost.GetIdentity().level == level)
            {
                return ghost;
            }
        }
        return null;
    }

    public int GetRandomGhostIndexByLevel(int level, GetGhostLevelFilterMode mode)
    {
        List<int> k = GetGhostsByLevel(level, mode);
        return k[Random.Range(0, k.Count)];
    }
    public GhostAI GetRandomGhostByLevel(int level, GetGhostLevelFilterMode mode)
    {
        List<int> k = GetGhostsByLevel(level, mode);
        return spawnedGhosts[k[Random.Range(0, k.Count)]];
    }
    #endregion

    #region Helper Functions (Server)
    /// <summary>
    /// Specifies options when finding a ghost
    /// </summary>
    public struct GetGhostOptions
    {
        //Need separate filtering for either ends of boolean value
        //As the complement can result in non-expect filter value thus true is required to start filter
        public bool filterGhostsWithoutPlayerTarget;
        public bool filterGhostsWithPlayerTarget;

        //Default constructor
        public GetGhostOptions(bool filterGhostsWithoutPlayerTarget = false, bool filterGhostsWithPlayerTarget = false)
        {
            this.filterGhostsWithoutPlayerTarget = filterGhostsWithoutPlayerTarget;
            this.filterGhostsWithPlayerTarget = filterGhostsWithPlayerTarget;
        }
    }
    /// <summary>
    /// Finds the Ghost closest to worldPos and writes out distance, if no spawned ghost returns null. Optionally specify options for filtering.
    /// </summary>
    /// <param name="worldPos"></param>
    /// <param name="distance"></param>
    /// <param name="options">Specifies specific filterings</param>
    /// <returns></returns>
    public GhostAI GetClosestGhost(Vector3 worldPos, out float distance, GetGhostOptions options = new GetGhostOptions())
    {
        if (spawnedGhosts.Count == 0)
        {
            distance = -1;
            return null;
        }

        GhostAI closest = null;
        float closestDistance = -1;
        for (int i = 0; i < spawnedGhosts.Count; i++)
        {
            float _distance = Vector3.Distance(spawnedGhosts[i].transform.position, worldPos);
            if (closest == null || _distance < closestDistance)
            {
                //Check if the current ghost satisfies filtering conditions
                if (InternalValidateGhost(spawnedGhosts[i], ref options))
                {
                    closestDistance = _distance;
                    closest = spawnedGhosts[i];
                }
            }
        }

        distance = closestDistance;
        return closest;
    }
    /// <summary>
    /// Returns the lowest health alive player, null if all players dead
    /// </summary>
    /// <returns></returns>
    public Player GetLowestHealthAlivePlayer()
    {
        GameObject[] playerArray = GameObject.FindGameObjectsWithTag("Player");
        Player minHealthPlayer = null;
        foreach (GameObject g in playerArray)
        {
            Player p = g.GetComponent<Player>();
            if (!p.is_alive) continue;
            if (minHealthPlayer == null || p.health < minHealthPlayer.health)
            {
                minHealthPlayer = p;
            }
        }

        return minHealthPlayer;
    }
    Transform FindNearestHMSpawnPointTransform(Vector3 targetPosition)
    {
        Transform closest = null;
        float closestDistance = float.MaxValue;
        foreach (Transform child in hostmachineManager.GetHMSpawnPoints().transform)
        {
            float dist = Vector3.Distance(child.position, targetPosition);
            if (dist < closestDistance)
            {
                closestDistance = dist;
                closest = child;
            }
        }

        return closest;
    }

    /// <summary>
    /// Gets a random point on NavMesh based on range search from current transform position
    /// </summary>
    /// <param name="position"></param>
    /// <param name="max_attempt"></param>
    /// <param name="range"></param>
    /// <param name="result"></param>
    /// <returns></returns>
    public bool SamplePosition(Vector3 position, float max_attempt, float range, out Vector3 result)
    {
        for (int i = 0; i < max_attempt; i++)
        {
            Vector2 randomVector2 = Random.insideUnitCircle * range;
            Vector3 randomPoint = new Vector3(position.x + randomVector2.x, position.y, position.z + randomVector2.y);
            NavMeshHit hit;
            if (NavMesh.SamplePosition(randomPoint, out hit, 1.0f, 0))
            {
                result = hit.position;
                return true;
            }
        }
        result = Vector3.zero;
        return false;
    }
    #endregion

    #region Command Testing Functions (Internal Command, Can call from client)
    [Command(requiresAuthority = false)]
    public void TInternalCMDSpawnGhostNearPosition(Vector3 position)
    {
        bool status = SpawnGhostNearPosition(0, position);
        if (!status)
        {
            Debug.LogError("Failed");
        }
    }

    void PRINTDB(string message)
    {
        if (!PRINT_DEBUG) return;
        Debug.Log(message);
    }
    #endregion
}
