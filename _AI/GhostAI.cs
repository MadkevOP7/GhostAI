// Copyright © 2024 by MADKEV Studio, all rights reserved

///Info: Default Animation Clip States:
///      attack
///      speed
///      idle
///      forceState (boolean) //To stop idle and speed transitions
///Override functions for specific AI implementations      
using System.Collections;
using System.Collections.Generic;
using Mirror;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.SceneManagement;
[RequireComponent(typeof(GhostIdentity))]
[RequireComponent(typeof(NetworkTransformReliable))]
[RequireComponent(typeof(NetworkAnimator))]

public class GhostAI : NetworkBehaviour
{
    #region Const/Static Defines
    public static string EFFECT_FOOT_PRINT = "GhostFootPrint";
    public static string EFFECT_HAND_PRINT = "GhostHandPrint";

    public const float FOOT_PRINT_SPACING_MIN = 1;
    public const float FOOT_PRINT_SPACING_MAX = 1.68f;
    public const int FOOT_PRINT_TRAIL_AMOUNT_MIN = 3;
    public const int FOOT_PRINT_TRAIL_AMOUNT_MAX = 8;
    public const int FOOT_PRINT_SPAWN_INTERVAL = 20; //Spawn trail every 20 seconds
    private const int ONLY_SYNC_IF_CHANGED_CORRECTION_MULTIPLIER = 2;
    #endregion

    #region Default Fields
    [Header("Setup")]
    GhostIdentity mIdentity;
    public Animator animator;
    public NavMeshAgent agent;
    public Transform head;
    [Header("Settings")]
    public float tickRate = 0.5f;
    public int attackDamage = 25;
    public int chargedAttackDamage = 100;
    public float attackDistance = 1.68f;
    public float visionDistance = 10f;
    public float losePlayerTime = 6f;
    private float targetReachedDistance = 1.86f;
    public float normalSpeed = 1.6f;
    public float runSpeed = 2.5f;
    public static string FORCE_STATE = "forceState";
    [Header("Cache settings")]
    public int obsCacheSize = 128;
    //Field of View
    [Header("Field Of View")]
    public LayerMask targetMask; //For players
    public LayerMask obstacleMask; //For walls/blocks etc
    protected LayerMask doorMask;

    [SerializeField]
    protected LayerMask excludeSelfMask; //Everything except Ghost layer
    [Range(0, 360)]
    public float viewAngle = 145f;
    static int DEFAULT_LEANING_PERCENTAGE = 35;
    #endregion

    #region Runtime Cache
    private WaitForSeconds mMainTickWaitForSeconds;
    #region State Timers
    //State timers are things that are in relation with time, but don't need coroutine since coroutine updates more frequently
    protected float mFootprintEffectTimer;
    #endregion
    Vector3 lastAttackDistance;
    GlobalWaypoint globalWaypoint;
    protected List<Transform> visibleTargets = new List<Transform>();
    protected List<NavMeshObstacle> obsCache = new List<NavMeshObstacle>();
    protected List<Transform> obstacles = new List<Transform>();
    protected float currentSpeed;
    public AITarget currentTarget;
    public AIAttackingTarget currentAttackTarget;
    public System.DateTime lastPlayerVisibleTimestamp;

    //Cache for time frame change, we cache originally inactive components
    //When player changes to past, need to enable things and disable originally disabled components
    private List<Renderer> originallyInactiveRendererCache = new List<Renderer>();
    private List<AudioSource> originallyInactiveAudioSourceCache = new List<AudioSource>();
    private List<Collider> originallyInactiveColliderCache = new List<Collider>();
    #endregion

    #region Coroutines Reference
    protected Coroutine AIPlaySingleAnimationCoroutine;
    protected Coroutine AIWatchForPlayerDuringPauseCoroutine;
    protected Coroutine AIWaypointReachedCoroutine;
    protected Coroutine AIDoorHandlingCoroutine;
    protected Coroutine AIAnimatedAttackCoroutine;
    protected Coroutine AIRotationCoroutine;
    protected Coroutine AIBrainCoroutine;
    protected Coroutine AITimedLosePlayerCoroutine;
    protected Coroutine AIProcessHidingSpotCoroutine;
    protected Coroutine AISpawnFootprintTrailCoroutine;
    #endregion

    #region DEBUG - EDITOR TIME
#if UNITY_EDITOR
    public DebugAIState DAIState;
    public enum DebugAIState
    {
        root,
        obs,
        wayPoint,
        movement,
        searchForPlayer,
        chasePlayer,
        attackPlayer,
        destroyBuilding,
        processDoor,
        processHidingSpot,
    }

    public bool DTargetVisible;
    public bool DTargetReachable;
    public bool DMainLoopRunning = true;
    public bool DLastPathState;
    public string DLastObstacleName;
#endif
    #endregion

    #region Initialization
    private void Awake()
    {
        if (animator == null)
        {
            animator = GetComponent<Animator>();
        }
    }

    public virtual void Start()
    {
        if (SceneManager.GetActiveScene().name == "Forest")
        {
            ForestManager.Instance.AddClient(transform);
        }
    }

    public override void OnStartClient()
    {
        base.OnStartClient();
        mIdentity = GetComponent<GhostIdentity>();
        //Subscribe to events
        BaseTimeframeManager.Instance.OnRefreshLocalPlayerIsPastState += OnLocalPlayerTimeframeChanged;
        BaseTimeframeManager.Instance.OnLocalPlayerLimenBreakoccured += OnLocalPlayerLimenBreakOccured;

        //Set initial state
        LocalSetCurrentTimeframeVisibility(BaseTimeframeManager.Instance.GetIsPastLocal());
    }
    public override void OnStartServer()
    {
        base.OnStartServer();
        if (agent == null)
        {
            agent = GetComponent<NavMeshAgent>();
        }

        agent.autoBraking = false;
        doorMask = LayerMask.GetMask("Door");

        //12/30/2023: We must also exclude the Player layer during obstacle search, or it could flag player as obstacle blockage
        excludeSelfMask = ~(LayerMask.GetMask("Ghost") | LayerMask.GetMask("Player") | LayerMask.GetMask("Ignore Raycast") | LayerMask.GetMask("UI") | LayerMask.GetMask("Post Processing") | LayerMask.GetMask("TransparentFX"));
        SetSpeed(normalSpeed);
        globalWaypoint = GlobalWaypoint.Instance;

        InitializeGhostAI();
        AIBrainCoroutine = StartCoroutine(TickAI(tickRate));
    }

    public virtual void InitializeGhostAI()
    {
        //Set obstacle mask to include BuildItem by default
        obstacleMask = (1 << LayerMask.NameToLayer("BuildItem"));
    }

    /// <summary>
    /// By default returns middle ground Vector3 between transform.position and head.position. Returns transform.position if head is null.
    /// </summary>
    public virtual Vector3 CalculateVisionPosition()
    {
        if (head == null) return transform.position;
        return new Vector3(transform.position.x, transform.position.y + ((head.position.y - transform.position.y) / 2), transform.position.z);
    }

    #endregion

    #region Callbacks
    public virtual void OnLocalPlayerLimenBreakOccured()
    {
        LocalSetCurrentTimeframeVisibility(true);
    }
    public virtual void OnLocalPlayerTimeframeChanged(bool isPast)
    {
        //If changed to the present and Limen Break has not occurred
        //We will lose our current target

        if (!isPast && !BaseTimeframeManager.Instance.GetHasLimenBreakOccuredLocal())
        {
            CMDForceLoseCurrentPlayerTarget();
        }

        LocalSetCurrentTimeframeVisibility(isPast);
    }
    #endregion

    #region Core Coroutine

    protected virtual IEnumerator TickAI(float rate)
    {
        mMainTickWaitForSeconds = new WaitForSeconds(rate);
        while (true)
        {
            yield return mMainTickWaitForSeconds;
            OnAITick();
        }
    }
    protected virtual IEnumerator TimedLosePlayer(float seconds)
    {
        yield return new WaitForSeconds(seconds);
        if (currentTarget == null || currentTarget.playerTargetCache == null)
        {
            ChangeTarget(null);
            AITimedLosePlayerCoroutine = null;
            yield break;
        }

        if (!ValidateTarget(currentTarget.targetTransform).HasVisibleTarget())
        {
            AITimedLosePlayerCoroutine = null;
            ChangeTarget(null);
            yield break;
        }

        AITimedLosePlayerCoroutine = null;
    }

    protected virtual IEnumerator SmoothRotation(Quaternion rotation)
    {
        float progress = 0;
        while (progress < 1)
        {
            progress += normalSpeed * Time.deltaTime;
            transform.rotation = Quaternion.Slerp(transform.rotation, rotation, progress);

            yield return null;
        }
        AIRotationCoroutine = null;
    }

    protected virtual IEnumerator AnimatedAttack()
    {
        PauseMainAILoop();
        FaceTarget(currentAttackTarget.targetTransform);
        PlaySingleAnimation("attack");
        yield return new WaitForEndOfFrame();

        //By default, we want to damage player during 1/2 of AI attack animation as that's when the arm swings down
        float attackAnimationDuration = animator.GetCurrentAnimatorStateInfo(0).length;
        float timeToDealDamage = attackAnimationDuration - attackAnimationDuration / 2;
        yield return new WaitForSeconds(timeToDealDamage);

        //Note: 09/24//2023 - Move to deal region specific as we want all players in the range to be damaged
        //For future, consider moving validation check directly to DealPlayerDamageSpecific()
        //if (IsAttackDistance(currentAttackTarget.targetTransform) && ValidateTarget(currentAttackTarget.targetTransform).HasVisibleTarget())
        //{
        //    DealPlayerDamageSpecific();
        //}

        DealPlayerDamageRegional();
        //Wait till remaining amount of animation finishes playing
        yield return new WaitForSeconds(attackAnimationDuration - timeToDealDamage);
        AIAnimatedAttackCoroutine = null;
        ResumeMainAILoop();
    }
    protected virtual IEnumerator AnimatedDoorHandling(Door door)
    {
        PauseMainAILoop();
#if UNITY_EDITOR
        DAIState = DebugAIState.processDoor;
#endif
        yield return new WaitForSeconds(Random.Range(1, 3));

        //Need another check here as player could have opened the door during AI wait period.
        if (!door.isOpened)
        {
            //Leave hand print effect
            BaseEffectsManager.Instance.SpawnEffectServer(EFFECT_HAND_PRINT, transform.position + new Vector3(0, agent.height / 2, 0), transform.forward, AIMath.Decide2());
            door.Interact();
        }

        AIDoorHandlingCoroutine = null;
        ResumeMainAILoop();
    }
    protected virtual IEnumerator ProcessWaypointReached()
    {
        PauseMainAILoop();
        yield return new WaitForSeconds(Random.Range(3, 8));
        SetWaypointTarget(currentTarget != null && currentTarget.waypointTargetCache != null ? globalWaypoint.GetNextWaypoint(currentTarget.waypointTargetCache) : globalWaypoint.GetRandomWaypoint());

        AIWaypointReachedCoroutine = null;
        ResumeMainAILoop();
    }

    protected virtual IEnumerator ProcessPlayerHiding()
    {
        //Condition check before proceeding
        if (!InternalNullCheck() || currentTarget.playerTargetCache == null || currentTarget.playerTargetCache.currentHidingSpot == null)
        {
            AIProcessHidingSpotCoroutine = null;
            yield break;
        }


        PauseMainAILoop();
        FaceTarget(currentTarget.targetTransform);

#if UNITY_EDITOR
        DAIState = DebugAIState.processHidingSpot;
#endif
        //Percentage to open doors and attack player
        int openHidingSpotPercentage = LIMENDefine.GetAIProbabilityPercent(GameManager.instance._currentDifficulty);

        //Modify percentage based on last seen player timestamp
        double lastSeenPlayerSeconds = (System.DateTime.UtcNow - lastPlayerVisibleTimestamp).TotalSeconds;
        float waitTime;
        if (lastSeenPlayerSeconds < 4)
        {
            openHidingSpotPercentage *= 2;
            waitTime = 2;
        }
        else
        {
            waitTime = Random.Range(5, 8);
        }

        yield return new WaitForSeconds(waitTime / 2);

        //Play scary audio (target rpc): ie. I SEE YOU!
        //Check for player disconnect before proceeding
        if (!InternalNullCheck() || currentTarget.playerTargetCache == null) yield break;

        currentTarget.playerTargetCache.TRPCPlayGhostNearHidingSpotAudio(currentTarget.playerTargetCache.GetComponent<NetworkIdentity>().connectionToClient);
        yield return new WaitForSeconds(waitTime / 2);

        //Decide if not to open hiding spot and scare player :)
        if (!InternalNullCheck()) yield break;
        if (AIMath.Decide2(openHidingSpotPercentage) && currentTarget.playerTargetCache.currentHidingSpot != null)
        {
            //Opens hiding spot
            currentTarget.playerTargetCache.currentHidingSpot.UnlockHidingSpot();
            currentTarget.playerTargetCache.TRPCPlayGhostScreamAudio(currentTarget.playerTargetCache.GetComponent<NetworkIdentity>().connectionToClient);
        }
        else
        {
            //AI choose to not open hiding spot, discard player and move to waypoint (leave)
            if (!ValidateTarget(currentTarget.targetTransform).HasVisibleTarget())
            {
                //Since we are clearing player target, we should also clear the timed lose player coroutine
                if (AITimedLosePlayerCoroutine != null)
                {
                    StopCoroutine(AITimedLosePlayerCoroutine);
                    AITimedLosePlayerCoroutine = null;
                }

                ChangeTarget(null);
            }
        }

        ResumeMainAILoop();
        AIProcessHidingSpotCoroutine = null;
    }
    /// <summary>
    /// This tick is irregular and only used when AI main loop is paused (animating seeking at a target destination without speed), but still want to maintain ability to detect player
    /// </summary>
    /// <returns></returns>
    protected virtual IEnumerator PausedPlayerDetectionTick()
    {
        while (true)
        {
            OnLookForPlayerTargetTick();
            yield return new WaitForSeconds(tickRate);
        }
    }

    /// <summary>
    /// Stops default animator states from playing, plays a clip and resumes default animator state when finished.
    /// </summary>
    /// <param name="clipName"></param>
    /// <returns></returns>
    protected virtual IEnumerator PlaySingleAnimationClip(string clipName)
    {
        StopDefaultAnimationStates();
        animator.Play(clipName);
        yield return new WaitForEndOfFrame();
        float animationDuration = animator.GetCurrentAnimatorStateInfo(0).length;
        yield return new WaitForSeconds(animationDuration);
        ResumeDefaultAnimationStates();
        AIPlaySingleAnimationCoroutine = null;
    }
    #endregion

    #region Core Ticks
    public virtual void OnAITick()
    {
        try
        {
#if UNITY_EDITOR
            DAIState = DebugAIState.root;
#endif
            //Restore speed if target is further or null
            if (currentTarget == null || currentTarget.targetTransform == null || !IsAttackDistance(currentTarget.targetTransform))
            {
                SetSpeed(normalSpeed);
            }

            OnOBSTick();
            OnEffectsTick();
            if (currentTarget == null || currentTarget.IsWaypointTarget())
            {
                OnAIWaypointTick();
            }
            else
            {
                if (currentTarget.playerTargetCache && currentTarget.playerTargetCache.is_alive)
                {
                    OnAIChasePlayerTick();
                }
                else
                {
                    ChangeTarget(null);
                }
            }
        }
        catch (System.NullReferenceException e)
        {
            //If null reference exception, clear current target
            ChangeTarget(null);
            return;
        }

    }
    public virtual void OnAIChasePlayerTick()
    {
#if UNITY_EDITOR
        DAIState = DebugAIState.chasePlayer;
#endif
        //Player is visible
        bool playerVisible = ValidateTarget(currentTarget.targetTransform).HasVisibleTarget();
        if (playerVisible)
        {
            //Reset lose player timer
            if (AITimedLosePlayerCoroutine != null)
            {
                StopCoroutine(AITimedLosePlayerCoroutine);
            }
            AITimedLosePlayerCoroutine = StartCoroutine(TimedLosePlayer(GetSceneMultiplayer() * Random.Range(12, 18)));

            //Update player visible timestamp, useful for hiding spot calculations and if AI should have seen player going in
            lastPlayerVisibleTimestamp = System.DateTime.UtcNow;
        }

        //Note: 09/24/2023 - To avoid AI attacking across floors, vision check needed to start attack
        if (IsAttackDistance(currentTarget.targetTransform)/* || Distance2D(lastAttackDistance, transform.position) < 0.3f*/)
        {
            FaceTarget(currentTarget.targetTransform);
            if (ValidateTarget(currentTarget.targetTransform).HasVisibleTarget())
            {
                SetPlayerAttackTarget(currentTarget.playerTargetCache);
                Attack();

                //Record each position after attack, as position may slightly change but we don't want AI to move again
                lastAttackDistance = transform.position;
                return;
            }
            else
            {
                //Player is not visible but in attack distance, check and handle if player is hiding in a hiding spot
                if (currentTarget.playerTargetCache.currentHidingSpot != null)
                {
                    if (AIProcessHidingSpotCoroutine == null)
                    {
                        AIProcessHidingSpotCoroutine = StartCoroutine(ProcessPlayerHiding());
                    }
                }
            }
        }
        else
        {
            if (currentTarget.playerTargetCache != null && currentTarget.playerTargetCache.currentHidingSpot != null)
            {
                //Player is hiding, modify movement distance
                if (IsWithinDistance(currentTarget.targetTransform, attackDistance) || IsWithinDistance(currentTarget.playerTargetCache.currentHidingSpot.transform, attackDistance))
                {
                    if (AIProcessHidingSpotCoroutine == null)
                    {
                        AIProcessHidingSpotCoroutine = StartCoroutine(ProcessPlayerHiding());
                    }
                    return;
                }
            }
            //Advance movement only when our distance is not close to player to avoid player pushing
            //Pathfinding reachable is handled through OnOBSTick()
            OnMovementTick(currentTarget.targetTransform);
        }

    }
    public virtual void OnAIWaypointTick()
    {
#if UNITY_EDITOR
        DAIState = DebugAIState.wayPoint;
#endif
        if (currentTarget == null)
        {
            SetWaypointTarget(globalWaypoint.GetRandomWaypoint());
        }

        if (ReachedCurrentTarget())
        {
            if (AIWaypointReachedCoroutine == null)
            {
                AIWaypointReachedCoroutine = StartCoroutine(ProcessWaypointReached());
            }
        }

        OnLookForPlayerTargetTick();
        OnMovementTick(currentTarget.targetTransform);
    }

    public virtual void OnDestroyBuildingTick()
    {
#if UNITY_EDITOR
        DAIState = DebugAIState.destroyBuilding;
#endif
        if (currentTarget == null || !currentTarget.IsValid()) return;
        //Remove closest obstacle till path becomes available
        if (HasUnreachableTarget())
        {
            if (currentAttackTarget == null && obstacles.Count > 0)
            {
                SetBuildingAttackTarget();
            }
        }
        else
        {

            //10/7/2023 - Not sure if works, but we should clear attack target building only, not clearing a player attack target here
            if (currentAttackTarget != null && currentAttackTarget.buildingCache != null)
            {
                currentAttackTarget = null;
            }
        }

        //Check if it's building type as we want to exclude player attack here
        if (currentAttackTarget != null && currentAttackTarget.buildingCache != null)
        {
            //Remove BuildItem from OBS Cache if it's already destroyed (isPickupMode)
            if (currentAttackTarget.buildingCache.isPickupMode)
            {
                RemoveOBSCache(obstacles.IndexOf(currentAttackTarget.buildingCache.transform));
                currentAttackTarget = null;
                return;

            }

            //If the picked BuildItem to attack is not path finding reachable, force select new one near AI since the previous one could be selected near Player.
            if (!ValidateTarget(currentAttackTarget.targetTransform).HasVisibleAndReachableTarget())
            {
                SetBuildingAttackTarget(true);
            }

            if (Distance2D(currentAttackTarget.targetTransform, transform) <= attackDistance)
            {
                FaceTarget(currentAttackTarget.targetTransform);
                Attack();
            }
            else
            {
                OnMovementTick(currentAttackTarget.targetTransform);
            }
        }
    }

    /// <summary>
    /// Handles door detection and NavmeshObstacle cache
    /// </summary>
    public virtual void OnOBSTick()
    {
#if UNITY_EDITOR
        DAIState = DebugAIState.obs;
#endif
        if (AIDoorHandlingCoroutine == null)
        {
            Collider[] doorsInView = Physics.OverlapSphere(transform.position, visionDistance, doorMask);
            foreach (var d in doorsInView)
            {
                if (IsInViewFrustum(d.transform.position) && Distance2D(transform, d.transform) <= targetReachedDistance && AIDoorHandlingCoroutine == null)
                {
                    Door door = d.GetComponentInParent<Door>();
                    if (!door.isHidingSpotDoor && (!door.isOpened || door.IsDoorMoving()))
                    {
                        StartCoroutine(AnimatedDoorHandling(door));
                        break;
                    }
                }
            }
        }

        Collider[] obstaclesInViewRadius = Physics.OverlapSphere(transform.position, visionDistance, obstacleMask);
        for (int i = 0; i < obstaclesInViewRadius.Length; i++)
        {
            Transform temp = obstaclesInViewRadius[i].transform.root != null ? obstaclesInViewRadius[i].transform.root : obstaclesInViewRadius[i].transform;
            AddOBSCache(temp, temp.GetComponent<NavMeshObstacle>());
        }

        if (HasUnreachableTarget())
        {
            OnTargetUnreachableTick();
        }
    }
    public virtual void OnLookForPlayerTargetTick()
    {
#if UNITY_EDITOR
        DAIState = DebugAIState.searchForPlayer;
#endif
        if (currentTarget != null && currentTarget.IsPlayerTarget()) return;

        visibleTargets.Clear();
        Collider[] targetsInViewRadius = Physics.OverlapSphere(transform.position, visionDistance, targetMask);
        for (int i = 0; i < targetsInViewRadius.Length; i++)
        {
            //Important! We need to check if player in alert distance is not currently in a hiding spot, as AI could be outside closet and wrongly adds player as visible target
            if ((IsAlertDistance(targetsInViewRadius[i].transform) && targetsInViewRadius[i].transform.root.GetComponent<Player>().currentHidingSpot == null) || ValidateTarget(targetsInViewRadius[i].transform).HasVisibleTarget())
            {
                Transform t = targetsInViewRadius[i].transform.root != null ? targetsInViewRadius[i].transform.root : targetsInViewRadius[i].transform;
                if (!visibleTargets.Contains(t))
                {
                    visibleTargets.Add(t);
                }
            }
        }

        if (visibleTargets.Count == 0)
        {
            return;
        }

        ProcessShouldChasePlayer(FindClosestTransform(visibleTargets));
    }

    /// <summary>
    /// By default ticks OnDestroyBuildingTick(). Note Door NavmeshObstacle has been removed to simple Raycast Check
    /// </summary>
    public virtual void OnTargetUnreachableTick()
    {
        OnDestroyBuildingTick();
    }

    /// <summary>
    /// Core tick for processing spawning effects like foot print, hand print or something
    /// </summary>
    public virtual void OnEffectsTick()
    {
        mFootprintEffectTimer += tickRate;
        if (mFootprintEffectTimer >= FOOT_PRINT_SPAWN_INTERVAL)
        {
            mFootprintEffectTimer = 0;
            ServerSpawnFootprintTrails();
        }
    }

    #endregion

    #region Break Main Functions - These pause main loop
    /// <summary>
    /// When AI is main loop is paused but want to maintain ability to detect player.
    /// </summary>
    public virtual void StartPausedPlayerWatch()
    {
        if (AIWatchForPlayerDuringPauseCoroutine == null)
        {
            AIWatchForPlayerDuringPauseCoroutine = StartCoroutine(PausedPlayerDetectionTick());
        }
    }

    /// <summary>
    /// Stop paused player detection watch.
    /// </summary>
    public virtual void StopPausedPlayerWatch()
    {
        if (AIWatchForPlayerDuringPauseCoroutine != null)
        {
            StopCoroutine(AIWatchForPlayerDuringPauseCoroutine);
            AIWatchForPlayerDuringPauseCoroutine = null;
        }
    }

    /// <summary>
    /// Pauses main AI Tick, useful for functions that may need pause AI logic (ie. Attack, Dashing, etc)
    /// </summary>
    public virtual void PauseMainAILoop(bool pauseSpeed = true, bool maintainPlayerWatch = true)
    {
        if (AIBrainCoroutine == null) return;
        if (maintainPlayerWatch)
        {
            StartPausedPlayerWatch();
        }
        if (pauseSpeed)
        {
            SetSpeed(0);
        }
        StopCoroutine(AIBrainCoroutine);
        AIBrainCoroutine = null;
#if UNITY_EDITOR
        DMainLoopRunning = false;
#endif
    }

    /// <summary>
    /// Resumes main AI Tick, useful for functions that may need pause AI logic (ie. Attack, Dashing, etc).
    /// Restores speed if player target is null or greater than attack distance, or specified forceResume = true
    /// </summary>
    public virtual void ResumeMainAILoop(bool forceResume = false)
    {
        if (AIBrainCoroutine != null) return;
        StopPausedPlayerWatch();

        //10/6/2023 Important: We need to flush the cache here to prevent AI stuck in process pause main loop (ie. door, or hide spot process)
        FlushCoroutineCache();
        //Restore speed if target is further
        if (forceResume || currentTarget == null || currentTarget.targetTransform == null || !IsAttackDistance(currentTarget.targetTransform))
        {
            SetSpeed(normalSpeed);
        }
        AIBrainCoroutine = StartCoroutine(TickAI(tickRate));
#if UNITY_EDITOR
        DMainLoopRunning = true;
#endif
    }

    /// <summary>
    /// By default does target validation and starts AnimatedAttack coroutine
    /// </summary>
    public virtual void Attack()
    {
        if (currentAttackTarget == null) return;

        //Pre-Attack target validation
        switch (currentAttackTarget.targetType)
        {
            case AIAttackingTarget.TargetType.Building:
                if (currentAttackTarget.buildingCache == null || currentAttackTarget.buildingCache.isPickupMode)
                {
                    RemoveOBSCache(obstacles.IndexOf(currentAttackTarget.targetTransform));
                    currentAttackTarget = null;
                    return;
                }
                break;

            case AIAttackingTarget.TargetType.Player:
                if (currentAttackTarget.playerCache == null || !currentAttackTarget.playerCache.is_alive)
                {
                    currentAttackTarget = null;
                    return;
                }
                break;
        }

        //Start attack sequence, animation and time events are handled through coroutine
        if (AIAnimatedAttackCoroutine == null)
        {
#if UNITY_EDITOR
            DAIState = DebugAIState.attackPlayer;
#endif
            AIAnimatedAttackCoroutine = StartCoroutine(AnimatedAttack());
        }
    }

    /// <summary>
    /// [Deprecated] By default subtracts player health by AI attack damage, this damage could only apply to the specific player that is current attacking target.
    /// To damage all players in the region, use DealPlayerDamageRegional()
    /// </summary>
    public virtual void DealPlayerDamageSpecific()
    {
        if (IsAttackDistance(currentAttackTarget.targetTransform) && ValidateTarget(currentAttackTarget.targetTransform).HasVisibleTarget())
            currentAttackTarget?.playerCache?.OnSanityHit(attackDamage);
    }

    /// <summary>
    /// Damages all players that are currently within attack distance and in enemy vision.
    /// </summary>
    public virtual void DealPlayerDamageRegional()
    {
        Collider[] targetsInViewRadius = Physics.OverlapSphere(transform.position, visionDistance, targetMask);
        List<Player> playerCache = new List<Player>();
        for (int i = 0; i < targetsInViewRadius.Length; i++)
        {
            if (IsAttackDistance(targetsInViewRadius[i].transform) && ValidateTarget(targetsInViewRadius[i].transform).HasVisibleTarget())
            {
                Transform t = targetsInViewRadius[i].transform.root != null ? targetsInViewRadius[i].transform.root : targetsInViewRadius[i].transform;
                Player p = t.GetComponent<Player>();
                if (!playerCache.Contains(p))
                {
                    p.OnSanityHit(attackDamage);
                    playerCache.Add(p);
                }
            }
        }
    }
    #endregion

    #region Core Processing Functions (Virtual)
    /// <summary>
    /// This function runs on the server only, used for starting and stopping Player chased audio.
    /// </summary>
    /// <param name="oldPlayer"></param>
    /// <param name="newPlayer"></param>
    [Server]
    public virtual void OnPlayerTargetChanged(Player oldPlayer, Player newPlayer)
    {
        if (oldPlayer != null)
        {
            oldPlayer.OnBeingChasedByAI(false, oldPlayer.gameObject, mIdentity);
        }

        if (newPlayer != null)
        {
            newPlayer.OnBeingChasedByAI(true, newPlayer.gameObject, mIdentity);
        }
    }

    /// <summary>
    /// Picks a build item (building piece) near player or near AI, depending on random option with DEFAULT_LEANING_PERCENTAGE leaning towards near player. 
    /// </summary>
    /// <param name="forceNearAI">Forces picking a build item near AI, useful if the previous near player pick is not path finding reachable.</param>
    public virtual void SetBuildingAttackTarget(bool forceNearAI = false)
    {
        if (obstacles == null || obstacles.Count == 0) return;
        currentAttackTarget = new AIAttackingTarget();
        Option option = forceNearAI ? Option.B : RandomOption(Option.A, DEFAULT_LEANING_PERCENTAGE);

        //Select build item near player
        if (option == Option.A)
        {
            currentAttackTarget.selectionMethod = AIAttackingTarget.SelectionMethod.nearPlayer;
            currentAttackTarget.targetTransform = FindClosestTransform(obstacles, currentTarget.targetTransform, true);
        }
        else
        {
            currentAttackTarget.selectionMethod = AIAttackingTarget.SelectionMethod.nearAI;
            currentAttackTarget.targetTransform = FindClosestTransform(obstacles);
        }

        currentAttackTarget.buildingCache = currentAttackTarget.targetTransform.GetComponent<BuildItem>();
    }

    /// <summary>
    /// Sets Player as attack target, will override current attack target if type isn't player.
    /// </summary>
    /// <param name="player"></param>
    public virtual void SetPlayerAttackTarget(Player player)
    {
        if (currentAttackTarget == null || currentAttackTarget.targetType != AIAttackingTarget.TargetType.Player)
        {
            currentAttackTarget = new AIAttackingTarget();
            currentAttackTarget.targetType = AIAttackingTarget.TargetType.Player;
            currentAttackTarget.targetTransform = player.transform;
            currentAttackTarget.playerCache = player;
        }
    }

    /// <summary>
    /// Decides if provided target (Player Transform) should be chased by AI. By default will immediately chase any player wit heath above 0.
    /// </summary>
    /// <param name="target"></param>
    public virtual void ProcessShouldChasePlayer(Transform target)
    {
        Player player = target.GetComponent<Player>();
        if (player != null && player.is_alive)
        {
            //We need to validate that player is in the same time frame as AI
            //Chase if limen break has happened, or player is in the past
            if (BaseTimeframeManager.Instance.GetHasLimenBreakOccuredLocal() || player.isPlayerInPast)
            {
                ChangeTarget(new AITarget(target, AITarget.TargetType.Player, player, null));
                ResumeMainAILoop();
            }
        }
    }

    #endregion

    #region Helper Functions
    /// <summary>
    /// Changes the current target and handles player chase audio
    /// </summary>
    /// <param name="target"></param>
    [Server]
    public void ChangeTarget(AITarget target)
    {
        OnPlayerTargetChanged(currentTarget?.playerTargetCache, target?.playerTargetCache);
        currentTarget = target;
    }

    [Server]
    private void ServerSpawnSingleFootprintEffect()
    {
        //Apply spawning with some offset to prevent going past a floor
        BaseEffectsManager.Instance.SpawnEffectServer(EFFECT_FOOT_PRINT, transform.position + new Vector3(0, agent.height / 2, 0), transform.forward, AIMath.Decide2());
    }

    /// <summary>
    /// [Server] Spawns a trail of footsteps 
    /// </summary>
    [Server]
    public void ServerSpawnFootprintTrails()
    {
        if (AISpawnFootprintTrailCoroutine != null)
        {
            StopCoroutine(AISpawnFootprintTrailCoroutine);
        }

        AISpawnFootprintTrailCoroutine = StartCoroutine(IEServerSpawnFootprintTrail(Random.Range(FOOT_PRINT_TRAIL_AMOUNT_MIN, FOOT_PRINT_TRAIL_AMOUNT_MAX + 1)));
    }
    /// <summary>
    /// Use to spawn several consecutive footprints that can form a foot print trail
    /// </summary>
    /// <param name="amount"></param>
    /// <returns></returns>
    IEnumerator IEServerSpawnFootprintTrail(int amount)
    {
        //A larger step size means the footprints will be spaced out more as the delay to spawn next one is more
        float stepSize = Random.Range(FOOT_PRINT_SPACING_MIN, FOOT_PRINT_SPACING_MAX);
        for (int i = 0; i < amount; i++)
        {
            ServerSpawnSingleFootprintEffect();
            yield return new WaitForSeconds(stepSize);
        }

        AISpawnFootprintTrailCoroutine = null;
    }

    /// <summary>
    /// Flushes (sets to null) pause main loop coroutines (Handling door, processing hiding spot)
    /// </summary>
    void FlushCoroutineCache()
    {
        if (AIDoorHandlingCoroutine != null)
        {
            StopCoroutine(AIDoorHandlingCoroutine);
            AIDoorHandlingCoroutine = null;
        }

        if (AIProcessHidingSpotCoroutine != null)
        {
            StopCoroutine(AIProcessHidingSpotCoroutine);
            AIProcessHidingSpotCoroutine = null;
        }
    }

    /// <summary>
    /// Returns true is current target is not null and current target transform is not null, false otherwise
    /// </summary>
    /// <returns></returns>
    bool InternalNullCheck()
    {
        if (currentTarget == null || currentTarget.targetTransform == null)
        {
            ChangeTarget(null);
            return false;
        }

        return true;
    }
    /// <summary>
    /// Used for playing animations like Attack, where it needs to exit out of idle and speed. Only played once
    /// </summary>
    public virtual void PlaySingleAnimation(string clipName)
    {
        if (AIPlaySingleAnimationCoroutine == null)
        {
            AIPlaySingleAnimationCoroutine = StartCoroutine(PlaySingleAnimationClip(clipName));
        }
    }

    public void StopDefaultAnimationStates()
    {
        animator.SetBool(FORCE_STATE, true);
    }
    public void ResumeDefaultAnimationStates()
    {
        animator.SetBool(FORCE_STATE, false);
    }
    /// <summary>
    /// Sets AI target to a waypoint if currentTarget isn't player
    /// </summary>
    /// <param name="waypoint"></param>
    protected void SetWaypointTarget(Waypoint waypoint)
    {
        if (currentTarget == null || currentTarget.targetType != AITarget.TargetType.Player)
        {
            ChangeTarget(new AITarget(waypoint.transform, AITarget.TargetType.Waypoint, null, waypoint));
        }
    }

    /// <summary>
    /// Assigns a player as Waypoint type, used for AIBrain setting player to be chased by AI quietly
    /// </summary>
    /// <param name="p"></param>
    public void SetPlayerWaypointTarget(Player p)
    {
        if (currentTarget == null || currentTarget.targetType != AITarget.TargetType.Player)
        {
            ChangeTarget(new AITarget(p.transform, AITarget.TargetType.Waypoint, null, null));
        }
    }

    /// <summary>
    /// Assigns a Host Machine as Waypoint type, used for AIBrain waypoint to machine
    /// </summary>
    /// <param name="hm"></param>
    public void SetHMWaypointTarget(HostMachine hm)
    {
        if (currentTarget == null || currentTarget.targetType != AITarget.TargetType.Player)
        {
            ChangeTarget(new AITarget(hm.transform, AITarget.TargetType.Waypoint, null, null));
        }
    }

    /// <summary>
    /// Checks whether a position in world space is within the view frustum of AI. This does not check for blockage and obstacles.
    /// </summary>
    /// <param name="position">Specified position in world space</param>
    /// <param name="checkDistance">Whether to check if target is within AI view distance range. True by default</param>
    /// <returns></returns>
    protected bool IsInViewFrustum(Vector3 position, bool checkDistance = true)
    {
        if (checkDistance && Distance2D(transform.position, position) > visionDistance) return false;

        //05/13/2024
        //We need to drop y axis to flatten the frustum check on a 2d plane, ignoring y differences
        Vector3 direction = position - transform.position;
        direction.y = 0;
        Vector3 fwdDropY = head.forward;
        fwdDropY.y = 0;
        return (Vector3.Angle(direction, fwdDropY) <= viewAngle);
    }
    protected bool IsInViewFrustum(Transform target)
    {
        return IsInViewFrustum(target.position);
    }

    /// <summary>
    /// Validates if a target is in view frustum, not blocked by obstacles and path finding reachable. Results are returned in a struct.
    /// </summary>
    /// <param name="target"></param>
    /// <returns></returns>
    protected AITargetValidationResult ValidateTarget(Transform target)
    {
        Vector3 tPos = target.position;

        if (IsInViewFrustum(tPos))
        {
            //09/10/2023, re-factored to InternalCheckForVisionBlockage() for more accurate multple ray cast check.
            //if (!Physics.Raycast(CalculateVisionPosition(), dirToTarget, out info, dstToTarget, excludeSelfMask))
            //{
            //    return new AITargetValidationResult(true, CanReachTarget(tPos), null);
            //}
            Transform _obstacle;
            if (InternalCheckForVisionBlockage(target, out _obstacle))
            {
#if UNITY_EDITOR
                DTargetReachable = CanReachTarget(tPos);
                DTargetVisible = true;
#endif
                return new AITargetValidationResult(true, CanReachTarget(tPos), true, null);
            }

#if UNITY_EDITOR
            DTargetReachable = CanReachTarget(tPos);
            DTargetVisible = false;
            DLastObstacleName = _obstacle.name;
#endif
            //Debug.LogError("AI Obstacle: " + _obstacle.transform.gameObject.name);
            return new AITargetValidationResult(false, CanReachTarget(tPos), true, _obstacle);
        }

#if UNITY_EDITOR
        DTargetReachable = CanReachTarget(tPos);
        DTargetVisible = false;
#endif
        return new AITargetValidationResult(false, CanReachTarget(tPos), false, null);
    }

    /// <summary>
    /// Internal function for validating if target is blocked by obstacle and cannot be seen by AI.
    /// This checks for partial blockage by offsetting multiple ray casts up/down, left/right from the ray directly to target.
    /// Increasing the number of rays could lead to higher accuracy but more performance hit.
    /// </summary>
    /// <param name="target"></param>
    /// <param name="obstacle"></param>
    /// <returns>true if target can be seen by AI, false otherwise</returns>
    protected virtual bool InternalCheckForVisionBlockage(Transform target, out Transform obstacle)
    {
        Vector3 toTarget = target.position - transform.position;

        // Define the number of ray casts and the angle step for horizontal and vertical scans.
        int horizontalRays = 5; // Number of horizontal rays
        int verticalRays = 3;   // Number of vertical rays

        float horizontalStep = 5.0f; // Angle step for horizontal rays
        float verticalStep = 15.0f;  // Angle step for vertical rays

        // Calculate the initial ray direction (straight towards the target).
        Vector3 initialRayDirection = toTarget.normalized;
        float dstToTarget = Vector3.Distance(transform.position, target.position);

        RaycastHit info;
        obstacle = null;
        for (int h = 0; h < horizontalRays; h++)
        {
            for (int v = 0; v < verticalRays; v++)
            {
                float horizontalOffset = (horizontalRays > 1) ? (-horizontalStep * 0.5f + h * horizontalStep / (horizontalRays - 1)) : 0.0f;
                float verticalOffset = (verticalRays > 1) ? (-verticalStep * 0.5f + v * verticalStep / (verticalRays - 1)) : 0.0f;

                Vector3 rayDirection = Quaternion.Euler(verticalOffset, horizontalOffset, 0) * initialRayDirection;


                if (!Physics.Raycast(CalculateVisionPosition(), rayDirection, out info, dstToTarget, excludeSelfMask))
                {
                    obstacle = null;
                    return true;
                }

                //Update obstacle, currently we update each time so the last obstacle will be returned
                obstacle = info.collider.transform;
            }
        }

        return false;
    }

    /// <summary>
    /// Returns true if currentTarget is not null and path finding unreachable.
    /// </summary>
    /// <returns></returns>
    protected bool HasUnreachableTarget()
    {
        if (currentTarget == null) return false;

        return !CanReachTarget(currentTarget.targetTransform.position);
    }

    /// <summary>
    /// Adds Navmesh Obstacles reference to cache where they contribute to AI path finding. For optimization, when cache exceeds obsCacheSize, first added reference will be removed for every new addition.
    /// </summary>
    /// <param name="t"></param>
    /// <param name="obs"></param>
    protected void AddOBSCache(Transform t, NavMeshObstacle obs)
    {
        if (t.GetComponent<BuildItem>().isPickupMode) return;
        if (!obs.enabled)
        {
            if (obsCache.Count < obsCacheSize)
            {
                obs.enabled = true;
                obsCache.Add(obs);
                obstacles.Add(t);
            }
            else
            {
                //Remove oldest cache and insert new one
                RemoveOBSCache(0);

                obs.enabled = true;
                obsCache.Add(obs);
                obstacles.Add(t);
            }
        }
    }

    /// <summary>
    /// Removes NavmeshObstacle reference from cache based on given index.
    /// </summary>
    /// <param name="index"></param>
    protected void RemoveOBSCache(int index)
    {
        if (index < 0 || index > obsCache.Count - 1) return;
        //obsCache[index].enabled = false;
        obsCache.RemoveAt(index);
        obstacles.RemoveAt(index);
    }

    /// <summary>
    /// Returns closest Transform in a list, null if list is null or empty.
    /// </summary>
    /// <param name="list"></param>
    /// <returns></returns>
    protected Transform FindClosestTransform(List<Transform> list)
    {
        return FindClosestTransform(list, transform, false);
    }

    /// <summary>
    /// Returns closest Transform in a list, null if list is null or empty. If validate = true, returns closest path finding reachable Transform, null if not found.
    /// </summary>
    /// <param name="list"></param>
    /// <param name="point"></param>
    /// <param name="validate"></param>
    /// <returns></returns>
    protected Transform FindClosestTransform(List<Transform> list, Transform point, bool validate)
    {
        if (list == null || list.Count == 0) return null;

        Transform _closest = list[0];
        float min = Distance2D(point, _closest);

        if (validate)
        {
            Transform _validT = null;
            foreach (Transform t in list)
            {
                if (CanReachTarget(t.position))
                {
                    _validT = t;
                    break;
                }
            }

            if (_validT)
            {
                _closest = _validT;
                min = Distance2D(point, _validT);
            }
            else
            {
                return null;
            }

        }

        //Find closest building block
        foreach (Transform t in list)
        {
            if (t == null) continue;
            float d = Distance2D(t, point);
            if (d < min)
            {
                if (validate)
                {
                    if (CanReachTarget(t.position))
                    {
                        min = d;
                        _closest = t;
                        continue;
                    }
                    continue;
                }
                else
                {
                    min = d;
                    _closest = t;
                }

            }
        }

        return _closest;
    }

    /// <summary>
    /// Returns true if target Transform has distance <= attackDistance.
    /// </summary>
    /// <param name="target"></param>
    /// <returns></returns>
    protected bool IsAttackDistance(Transform target)
    {
        return Distance2D(target, transform) <= attackDistance ? true : false;
    }

    /// <summary>
    /// Returns true if target Transform has distance <= distance
    /// </summary>
    /// <param name="target"></param>
    /// <param name="distance"></param>
    /// <returns></returns>
    protected bool IsWithinDistance(Transform target, float distance)
    {
        return Distance2D(target, transform) <= distance ? true : false;
    }

    /// <summary>
    /// Returns true if Player is in very close to the AI, by default visionDistance/2. Even if player is not in view frustum, AI would be alerted.
    /// </summary>
    /// <param name="target"></param>
    /// <returns></returns>
    protected virtual bool IsAlertDistance(Transform target)
    {
        return Vector3.Distance(target.position, transform.position) <= (visionDistance / 2) ? true : false;
    }


    /// <summary>
    /// Returns true if currentTarget isn't null and has distance to AI <= targetReachedDistance
    /// </summary>
    /// <returns></returns>
    protected bool ReachedCurrentTarget()
    {
        if (currentTarget == null) return false;
        return Distance2D(currentTarget.targetTransform, transform) <= targetReachedDistance ? true : false;
    }

    /// <summary>
    /// Returns true if successfully sampled a Navmesh point near AI in the range provided. Vector3 of successful sample returned through out parameter. 
    /// </summary>
    /// <param name="range">Range of sampling.</param>
    /// <param name="result">Vector3 of sample result, returns Vector3.zero if unsuccessful</param>
    /// <returns></returns>
    protected bool SampleRandomNextPoint(float range, out Vector3 result)
    {
        for (int i = 0; i < 30; i++)
        {
            Vector3 randomPoint = transform.position + Random.insideUnitSphere * range;
            NavMeshHit hit;
            if (NavMesh.SamplePosition(randomPoint, out hit, 1.0f, NavMesh.AllAreas))
            {
                result = hit.position;
                return true;
            }
        }
        result = Vector3.zero;
        return false;
    }

    /// <summary>
    /// Returns 2D distance between two Vectors, ignores Y.
    /// </summary>
    /// <param name="a"></param>
    /// <param name="b"></param>
    /// <returns></returns>
    protected float Distance2D(Vector3 a, Vector3 b)
    {
        float xDiff = a.x - b.x;
        float zDiff = a.z - b.z;
        return Mathf.Sqrt((xDiff * xDiff) + (zDiff * zDiff));
    }

    /// <summary>
    /// Returns 2D distance between two Transforms, ignores Y.
    /// </summary>
    /// <param name="a"></param>
    /// <param name="b"></param>
    /// <returns></returns>
    protected float Distance2D(Transform a, Transform b)
    {
        float xDiff = a.position.x - b.position.x;
        float zDiff = a.position.z - b.position.z;
        return Mathf.Sqrt((xDiff * xDiff) + (zDiff * zDiff));
    }

    /// <summary>
    /// Returns true if a Vector3 position can be reached by AI.
    /// </summary>
    /// <param name="pos"></param>
    /// <returns></returns>
    protected bool CanReachTarget(Vector3 pos)
    {
        NavMeshPath p = new NavMeshPath();
        return (agent.CalculatePath(pos, p) && p.status == NavMeshPathStatus.PathComplete);
    }

    /// <summary>
    /// Clears OBS Cache (both the Transform list and NavmeshObstacle list).
    /// </summary>
    protected void ResetOBSCache()
    {
        foreach (NavMeshObstacle nmo in obsCache)
        {
            nmo.enabled = false; ;
        }
        obsCache.Clear();
        obstacles.Clear();
    }
    #endregion

    #region Debug Functions
    public enum LogType
    {
        stateChange,
        fallBack,
        simple
    }
    public void AILog(string msg, LogType type)
    {
        if (type == LogType.simple)
        {
            Debug.Log("AI: [" + name + "] " + msg);
        }
        else if (type == LogType.stateChange)
        {
            Debug.Log("AI: [" + name + "] state changed to " + msg);
        }
        else if (type == LogType.fallBack)
        {
            Debug.Log("AI: [" + name + "] path fallback, reason: " + msg);
        }
    }
    #endregion

    #region Decision Making

    /// <summary>
    /// Returns different multiplier value depending on Scene for different AI behaviors (probability of picking a certain option).
    /// </summary>
    /// <returns></returns>
    public virtual float GetSceneMultiplayer()
    {
        if (SceneManager.GetActiveScene().name == "Forest")
        {
            return 1.5f;
        }

        return 1;
    }

    protected enum Option
    {
        A,
        B
    }

    /// <summary>
    /// Returns a random option (A/B) with leaning to specified leaning side of DEFAULT_LEANING_PERCENTAGE.
    /// </summary>
    /// <param name="leaningSide"></param>
    /// <returns></returns>
    protected Option RandomOption(Option leaningSide)
    {
        return RandomOption(leaningSide, DEFAULT_LEANING_PERCENTAGE);
    }

    /// <summary>
    /// Returns a random option (A/B) with leaning to specified leaning side of specified leaning percentage.
    /// </summary>
    /// <param name="leaningSide"></param>
    /// <param name="leaningPercentage"></param>
    /// <returns></returns>
    protected Option RandomOption(Option leaningSide, float leaningPercentage)
    {
        float rand = Random.Range(0f, 1f);
        float c = leaningPercentage / 100;
        if (rand < 0.5f + c)
        {
            return leaningSide;
        }
        else
        {
            if (leaningSide == Option.A)
            {
                return Option.B;
            }
            else
            {
                return Option.A;
            }
        }
    }
    #endregion

    #region Movement
    public virtual void SetSpeed(float _speed)
    {
        agent.speed = _speed;
        currentSpeed = _speed;
        animator.SetFloat("speed", _speed / runSpeed);
    }

    public virtual void StopMovement()
    {
        if (!agent.isStopped)
        {
            agent.isStopped = true;
            SetSpeed(0);
        }
    }

    public virtual void ResumeMovement()
    {
        if (agent.isStopped)
        {
            agent.isStopped = false;
            SetSpeed(normalSpeed);
        }
    }

    public virtual void OnMovementTick(Transform target)
    {

        ResumeMovement();
        //Path recalculation is needed as target could be moving (ie. Player)
        bool state = agent.SetDestination(target.position);
#if UNITY_EDITOR
        DAIState = DebugAIState.movement;
        DLastPathState = state;
#endif
    }

    public void FaceTarget(Transform target)
    {
        Vector3 lookPos = target.position - transform.position;
        lookPos.y = 0;
        Quaternion rotation = Quaternion.LookRotation(lookPos);
        if (AIRotationCoroutine != null)
        {
            StopCoroutine(AIRotationCoroutine);
        }
        AIRotationCoroutine = StartCoroutine(SmoothRotation(rotation));
    }

    #endregion

    #region Debug View
    public Vector3 DirFromAngle(float angleInDegrees, bool angleIsGlobal)
    {
        if (!angleIsGlobal)
        {
            angleInDegrees += transform.eulerAngles.y;
        }
        return new Vector3(Mathf.Sin(angleInDegrees * Mathf.Deg2Rad), 0, Mathf.Cos(angleInDegrees * Mathf.Deg2Rad));
    }
    #endregion

    #region Data Structure
    public class AIAttackingTarget
    {
        public Transform targetTransform;
        public enum TargetType
        {
            Building,
            Player
        }
        public TargetType targetType;

        public enum SelectionMethod
        {
            nearAI,
            nearPlayer
        }
        public SelectionMethod selectionMethod;
        public BuildItem buildingCache;
        public Player playerCache;
    }
    public class AITarget
    {
        public AITarget(Transform target, TargetType type, Player playerCache, Waypoint waypointCache)
        {
            targetTransform = target;
            targetType = type;
            playerTargetCache = playerCache;
            waypointTargetCache = waypointCache;
        }
        public enum TargetType
        {
            Waypoint,
            Player
        }

        public TargetType targetType;

        public Transform targetTransform;
        public Waypoint waypointTargetCache;
        public Player playerTargetCache;

        public bool IsWaypointTarget()
        {
            return targetType == TargetType.Waypoint;
        }

        public bool IsPlayerTarget()
        {
            return targetType == TargetType.Player;
        }

        public bool IsValid()
        {
            return targetTransform != null;
        }
    }

    public struct AITargetValidationResult
    {
        public bool isTargetVisible;
        public bool isTargetReachable;

        //This bool is added to differentiate between frustum check visible and blockage despite in frustum
        public bool isTargetInViewFrustum;
        //if target is blocked by something (checked by ray cast hit)
        public Transform obstacle;

        //Default Constructor
        public AITargetValidationResult(bool _isTargetVisible, bool _isTargetReachable, bool _isTargetInViewFrustum, Transform _obstacle) : this()
        {
            this.isTargetVisible = _isTargetVisible;
            this.isTargetReachable = _isTargetReachable;
            this.isTargetInViewFrustum = _isTargetInViewFrustum;
            this.obstacle = _obstacle;
        }

        //Returns true if target is in view frustum. Does not check for path finding target reachable.
        public bool HasVisibleTarget()
        {
            return isTargetVisible;
        }

        //Checks path finding target reachable.
        public bool HasVisibleAndReachableTarget()
        {
            return isTargetVisible && isTargetReachable;
        }
    }
    #endregion

    #region Auto Validate Components
    private void OnValidate()
    {
        var NT = GetComponent<NetworkTransformReliable>();
        NT.syncDirection = SyncDirection.ServerToClient;
        NT.compressRotation = true;
        NT.onlySyncOnChange = true;
        NT.onlySyncOnChangeCorrectionMultiplier = ONLY_SYNC_IF_CHANGED_CORRECTION_MULTIPLIER;

        var NA = GetComponent<NetworkAnimator>();
        NA.syncDirection = SyncDirection.ServerToClient;
        NA.clientAuthority = false;
        if (NA.animator == null)
        {
            NA.animator = GetComponent<Animator>();
        }

        if (!head)
            Debug.LogError(gameObject.name + " is missing head Transform! This must be set!");

    }
    #endregion

    #region Timeframe Response
    /// <summary>
    /// Warning: This function is used for Time frame state handling only
    /// Forces AI to clear current player target
    /// Note: AI runs on the server, however, time frame change is local, thus during local callback
    /// We need to inform server to lose the current target
    /// </summary>
    [Command(requiresAuthority = false)]
    private void CMDForceLoseCurrentPlayerTarget()
    {
        //Stop existing attacks
        if (AIAnimatedAttackCoroutine != null)
        {
            StopCoroutine(AIAnimatedAttackCoroutine);
            AIAnimatedAttackCoroutine = null;
        }

        if (currentAttackTarget != null && currentAttackTarget.targetType == AIAttackingTarget.TargetType.Player)
        {
            if (currentTarget.playerTargetCache != null)
            {
                currentAttackTarget.playerCache.GetComponent<CameraManager>().TRPCUnGlitchCamera();
            }
            currentAttackTarget = null;
        }

        if (currentTarget != null && currentTarget.playerTargetCache != null)
        {
            ChangeTarget(null);
        }

        ResumeMainAILoop(true);
    }
    /// <summary>
    /// Warning: This is strictly used for time frame changes (past/present) change
    /// Changes by visibility are local, non-networked
    /// Changes include enable/disable MeshRenderer, AudioSource, and Colliders
    /// Note: This function is local, runs on client
    /// </summary>
    /// <param name="visible"></param>
    public virtual void LocalSetCurrentTimeframeVisibility(bool visible)
    {
        if (!visible)
        {
            //We are going to be disabling components, clear cache as we will need recompute
            originallyInactiveRendererCache.Clear();
            originallyInactiveAudioSourceCache.Clear();
            originallyInactiveColliderCache.Clear();
        }

        //GetComponent<T>(bool includeInactive) -> the include inactive is for searching on inactive GameObjects
        //Disabled components can still be found. We currently include only on active GameObjects
        //As disabled GameObjects won't have effect anyways
        foreach (Renderer renderer in GetComponentsInChildren<Renderer>())
        {
            if (!visible && !renderer.enabled)
            {
                //If we are going to set things to disabled, and this component is already disabled, add to originallyDisabled cache
                originallyInactiveRendererCache.Add(renderer);
            }

            renderer.enabled = visible;
        }

        foreach (AudioSource audio in GetComponentsInChildren<AudioSource>())
        {
            if (!visible && !audio.enabled)
            {
                //If we are going to set things to disabled, and this component is already disabled, add to originallyDisabled cache
                originallyInactiveAudioSourceCache.Add(audio);
            }

            audio.enabled = visible;
        }

        foreach (Collider collider in GetComponentsInChildren<Collider>())
        {
            if (!visible && !collider.enabled)
            {
                //If we are going to set things to disabled, and this component is already disabled, add to originallyDisabled cache
                originallyInactiveColliderCache.Add(collider);
            }

            collider.enabled = visible;
        }

        if (visible)
        {
            //If we just set everything to visible, we need to re-disable originally inactive components
            foreach (Renderer renderer in originallyInactiveRendererCache)
            {
                renderer.enabled = false;
            }

            foreach (AudioSource audio in originallyInactiveAudioSourceCache)
            {
                audio.enabled = false;
            }

            foreach (Collider collider in originallyInactiveColliderCache)
            {
                collider.enabled = false;
            }
        }
    }
    #endregion

    #region Getters
    public GhostIdentity GetIdentity()
    {
        return mIdentity;
    }
    public bool GetIsChasingPlayer()
    {
        return (currentTarget != null && currentTarget.targetType == AITarget.TargetType.Player && currentTarget.playerTargetCache.is_alive) || currentAttackTarget != null && currentAttackTarget.playerCache.is_alive;
    }
    public float GetAttackDamage()
    {
        return attackDamage;
    }

    /// <summary>
    /// Returns true if has a player target and target is alive, turn aliveOnly to optionally return dead player target.
    /// Note: Dead player target will be discarded next tick, thus mostly no reason to get it
    /// </summary>
    /// <param name="aliveOnly"></param>
    /// <returns></returns>
    public virtual bool GetHasPlayerTarget(bool aliveOnly = true)
    {
        Player player = GetPlayerTarget();
        return player != null && (!aliveOnly || player.is_alive);
    }

    /// <summary>
    /// Returns current player target if any, null otherwise
    /// </summary>
    /// <returns></returns>
    public virtual Player GetPlayerTarget()
    {
        if (currentTarget == null) return null;
        return currentTarget.playerTargetCache;
    }
    #endregion

    #region Public Control Functions
    /// <summary>
    /// Forces setting waypoint target, even if currently has unreached waypoint assigned or has player chasing target
    /// </summary>
    [Server]
    public void ForceSetWaypointTarget(Waypoint waypoint)
    {
        ChangeTarget(new AITarget(waypoint.transform, AITarget.TargetType.Waypoint, null, waypoint));
    }
    #endregion
}

