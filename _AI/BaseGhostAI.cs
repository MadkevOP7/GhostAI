// Copyright © 2024 by MADKEV Studio, all rights reserved

using UnityEngine;
using Mirror;
using System.Collections.Generic;
using Animancer;
using System;
using System.Reflection;
using System.Linq;
using Pathfinding;

[RequireComponent(typeof(GhostIdentity))]
[RequireComponent(typeof(NetworkTransformReliable))]
[RequireComponent(typeof(AnimancerComponent))]
[RequireComponent(typeof(Animator))]
[RequireComponent(typeof(NetworkAnimancer))]
[RequireComponent(typeof(FollowerEntity))]
public abstract class BaseGhostAI : NetworkBehaviour
{
    protected const int ONLY_SYNC_IF_CHANGED_CORRECTION_MULTIPLIER = 2;

    public AnimationClip[] GetAnimationClips() { return AnimationClips; }
    [Header("Animation Setup")]
    [Tooltip("Put all animations here to be able to play over network")]
    [SerializeField]
    protected AnimationClip[] AnimationClips;

    #region Public Interface External Functions (Abstract)

    /// <summary>
    /// Returns the GhostIdentity component which contains info about the current ghost.
    /// </summary>
    /// <returns></returns>
    public abstract GhostIdentity GetIdentity();

    /// <summary>
    /// Returns the current Player target. If mMovingTowardsPlayerTarget is True, AI will move towards this.
    /// </summary>
    /// <returns></returns>
    public abstract Player GetPlayerTarget();

    /// <summary>
    /// Returns the current Waypoint target. If mMoveTowardsDestination is True, and mMovingTowardsPlayerTarget AI will move towards this.
    /// </summary>
    /// <returns></returns>
    public abstract Waypoint GetWaypointTarget();

    /// <summary>
    /// Returns the timer of the current state. Timer is reset when state changes to a different state.
    /// </summary>
    /// <returns></returns>
    public abstract float GetCurrentStateTimer();

    /// <summary>
    /// Requests the AI to lose/stop chasing the current player target. This is useful for mechanics that involve offering
    /// spatial protection, such as Fulu Protector.
    /// <para>Note: This does not guarantee that AI will proceed with the request, whether or not AI does lose the target is based on each unique implementation.</para>
    /// </summary>
    public abstract void RequestToLoseCurrentPlayerTarget();
    #endregion

    private static List<BaseGhostAI> AllGhosts = new List<BaseGhostAI>();

    public static IEnumerable<BaseGhostAI> GetAllGhosts()
    {
        return AllGhosts;
    }

    protected virtual void Awake()
    {
        AllGhosts.Add(this);
    }

    protected virtual void OnDestroy()
    {
        AllGhosts.Remove(this);
    }
}

public abstract class BaseGhostAI<TState> : BaseGhostAI where TState : System.Enum
{
    public static bool DEBUG_LOG_ON = true;
    public static bool DEBUG_DISPLAY_ON = false;
    #region DATA STRUCTURE
    public class AISettings
    {
        public TState StartingState;
        [Tooltip("The angle in degrees of AI's field of view")]
        public float VisionAngle = 120;
        [Tooltip("The vision distance of AI.")]
        public float VisionDistance = 10;
        [Tooltip("The distance AI would be alerted if target is within, even if not visible or seen.")]
        public float AlertDistance = 3;
        [Tooltip("The distance AI is considered to have reached a target.")]
        public float TargetReachedDistance = 3;
    }

    public enum MovementState
    {
        // No movement = AI agent would be stopped and would not move to destination
        NoMovement,
        // Destination is updated each tick with player target's position and AI would start moving towards destination
        MoveTowardsPlayerTarget,
        // Destination is updated each tick with waypoint target's position and AI would start moving towards destination
        MoveTowardsWaypointTarget,

    }
    #endregion

    // CONST DEFINES
    public const int NUM_VISION_RAYS = 5;
    public const string FUNC_PREFIX_ON_STATE_ENTER = "OnStateEnter";
    public const string FUNC_PREFIX_ON_STATE_EXIT = "OnStateExit";
    public const string FUNC_PREFIX_ON_STATE_TICK = "OnStateTick";
    public static string EFFECT_FOOT_PRINT = "GhostFootPrint";
    public static string EFFECT_HAND_PRINT = "GhostHandPrint";
    protected const float TICK_INTERVAL = 0.2f;
    [Header("AI Component Reference")]
    public Transform mHeadBone;

    // AI VARIABLES - DEEFINED BY INHERITED IMPLEMENTATION FOR CUSTOMIZATION
    protected AISettings mAISetting;
    protected LayerMask mObstacleMask;
    protected LayerMask mPlayerMask;
    protected LayerMask mDoorMask;
    // COMPONENT REFERENCES
    private GhostIdentity mIdentity;
    public sealed override GhostIdentity GetIdentity() { return mIdentity; }
    private FollowerEntity mAgent;
    private AnimancerComponent mAnimancer;
    /// <summary>
    /// Returns the Animancer component, available on client and server.
    /// </summary>
    /// <returns></returns>
    protected AnimancerComponent GetAnimancer() { return mAnimancer; }
    // RUNTIME CONTROL VARIABLES
    /// <summary>
    /// Returns the current movement state. To change movement state (no movement, or move to a type of target),
    /// use SetMovementState.
    /// </summary>
    /// <returns></returns>
    protected MovementState GetMovementState() { return mMovementState; }
    private MovementState mMovementState = MovementState.NoMovement;
    /// <summary>
    /// Set's what AI should move towards. No movement, move to player target, or move to waypoint target.
    /// <para>Note: If movement state is set to Waypoint or Player, but their target cache is null,
    /// AI would remain in that state but not move until the target is set.</para>
    /// </summary>
    /// <param name="state"></param>
    protected void SetMovementState(MovementState state)
    {
        mMovementState = state;
    }

    private TState mCurrentAIState;

    /// <summary>
    /// Returns the current AI state. It's your responsibility to define the states enum in your AI implementation.
    /// </summary>
    /// <returns></returns>
    public TState GetCurrentState() { return mCurrentAIState; }
    // RUNTIME NON-CONTROL VARIABLES
    private float mMainLoopTimer = TICK_INTERVAL;
    private Vector3 mDestination;
    public sealed override Player GetPlayerTarget() { return mPlayerTarget; }
    private Player mPlayerTarget;
    /// <summary>
    /// Set's the Player target cache. Note: Setting this doesn't mean AI would be moving to the player. 
    /// Destination is determined by mMovementState variable.
    /// </summary>
    /// <param name="player">The player to be set as target.</param>
    protected void SetPlayerTarget(Player player)
    {
        mPlayerTarget = player;
    }
    public sealed override Waypoint GetWaypointTarget() { return mWaypointTarget; }
    private Waypoint mWaypointTarget;
    /// <summary>
    /// Set's the Waypoint target cache. Note: Setting this doesn't mean AI would be moving to the waypoint. 
    /// Destination is determined by mMovementState variable.
    /// </summary>
    /// <param name="waypoint">The waypoint to be set as target.</param>
    protected void SetWaypointTarget(Waypoint waypoint)
    {
        mWaypointTarget = waypoint;
    }

    private float mCurrentStateTimer;
    // Cache for time frame change, we cache originally inactive components
    // When player changes to past, need to enable things and disable originally disabled components
    private List<Renderer> mOriginallyInactiveRendererCache = new List<Renderer>();
    private List<AudioSource> mOriginallyInactiveAudioSourceCache = new List<AudioSource>();
    private List<Collider> mOriginallyInactiveColliderCache = new List<Collider>();
    // Stores if each implementation contains a method of that name, so before Invoke we don't need to check each time
    private Dictionary<string, bool> mStateInvokeHasMethodDictionary = new Dictionary<string, bool>();
    // Effect State
    private bool mPreviousFootstepFlipX = false;
    public sealed override float GetCurrentStateTimer() { return mCurrentStateTimer; }

    #region Internal Core - These are not exposed to AI implementations
    protected override void Awake()
    {
        base.Awake();
        mAnimancer = GetComponent<AnimancerComponent>();
        mIdentity = GetComponent<GhostIdentity>();
        mObstacleMask = ~(LayerMask.GetMask("Ghost") | LayerMask.GetMask("Player") | LayerMask.GetMask("Ignore Raycast") | LayerMask.GetMask("UI") | LayerMask.GetMask("Post Processing") | LayerMask.GetMask("TransparentFX"));
        mPlayerMask = LayerMask.GetMask("Player");
        mDoorMask = LayerMask.GetMask("Door");
    }

    /// <summary>
    /// Validates essential components
    /// </summary>
    protected override void OnValidate()
    {
        base.OnValidate();
        var NT = GetComponent<NetworkTransformReliable>();
        NT.syncDirection = SyncDirection.ServerToClient;
        NT.compressRotation = true;
        NT.onlySyncOnChange = true;
        NT.onlySyncOnChangeCorrectionMultiplier = ONLY_SYNC_IF_CHANGED_CORRECTION_MULTIPLIER;

        Animator animator = GetComponent<Animator>();
        GetComponent<AnimancerComponent>().Animator = animator;

        if (!mHeadBone)
            Debug.LogError($"{gameObject.name} is missing head Transform! This must be set!");
    }

    /// <summary>
    /// [Must Call Base] Base handles getting component and initializing necessary setup functions
    /// </summary>
    public override void OnStartServer()
    {
        base.OnStartServer();
        mAgent = GetComponent<FollowerEntity>();
        mAISetting = OnInitializeAISettings();
        AIBrain.Instance.OnHostMachineAttackedCallbackServer += OnHostMachineAttackedServer;
        AIBrain.Instance.OnHostMachineDestroyedCallbackServer += OnHostMachineDestroyedServer;

        // Since defined state starts as default, we need to manually invoke the OnStateEnter function
        // for that default state
        InvokeSafe($"{FUNC_PREFIX_ON_STATE_ENTER}{mCurrentAIState}");

#if UNITY_EDITOR
        DEBUG_LOG_ON = true;
#endif
    }

    /// <summary>
    /// [Must Call Base] Base handles subscription to handle visibility changes in response to local client changing time frame
    /// </summary>
    public override void OnStartClient()
    {
        base.OnStartClient();
        BaseTimeframeManager.Instance.OnRefreshLocalPlayerIsPastState += HandleAIVisibilityChangeClient;
        BaseTimeframeManager.Instance.OnLocalPlayerLimenBreakoccured += () => HandleAIVisibilityChangeClient(true);
    }

    private void Update()
    {
        // AI only ticks on the server
        if (!isServer) return;

        if (mMainLoopTimer >= TICK_INTERVAL)
        {
            mMainLoopTimer = 0;
            OnAITickServer();
        }

        // Increment timer, state timer incrementation goes here instead of the OnAITickServer function
        // Because there Time.deltaTime isn't correct as tick is only every TICK_INTERVAL
        mMainLoopTimer += Time.deltaTime;
        mCurrentStateTimer += Time.deltaTime;
        if (DEBUG_LOG_ON)
            DisplayDebugInfo();
    }

    /// <summary>
    /// Use this function to print out a debug log. This function will add the AI's name before the log statement.
    /// </summary>
    /// <param name="message"></param>
    private void LogDebug(string message)
    {
        if (DEBUG_LOG_ON)
            Debug.Log($"AI: {gameObject.name} Log: {message}");
    }

    public sealed override void RequestToLoseCurrentPlayerTarget()
    {
        OnLosePlayerTargetRequested();
        LogDebug($"AI {gameObject.name}: Received request to lose current player target {(GetPlayerTarget() != null ? GetPlayerTarget().gameObject.name + " Is valid target: " + ValidatePlayerTarget() : "No player target")}");
    }

    #endregion

    #region Internal Helpers

    /// <summary>
    /// Checks if a point is near a line segment within a threshold.
    /// </summary>
    /// <param name="point">The point to check.</param>
    /// <param name="segmentStart">The start of the line segment.</param>
    /// <param name="segmentEnd">The end of the line segment.</param>
    /// <param name="threshold">The maximum distance from the line segment.</param>
    /// <returns>True if the point is near the segment; otherwise, false.</returns>
    private bool IsPointNearLineSegment(Vector3 point, Vector3 segmentStart, Vector3 segmentEnd, float threshold)
    {
        Vector3 line = segmentEnd - segmentStart;
        Vector3 projection = Vector3.Project(point - segmentStart, line);
        Vector3 closestPointOnLine = segmentStart + projection;

        // Ensure the closest point is within the line segment
        float lineLengthSquared = line.sqrMagnitude;
        float dotProduct = Vector3.Dot(projection, line);

        if (dotProduct < 0 || projection.sqrMagnitude > lineLengthSquared)
        {
            // Closest point is outside the segment, check end points
            return Vector3.Distance(point, segmentStart) <= threshold || Vector3.Distance(point, segmentEnd) <= threshold;
        }

        // Closest point is on the segment
        return Vector3.Distance(point, closestPointOnLine) <= threshold;
    }

    #endregion

    #region EXPOSED IMPLEMENTATIONS
    /// <summary>
    /// [Must Implement] Define and return an AISettings struct for customizing your AI.
    /// Initialize other variables declared for your AI implementation here as well.
    /// </summary>
    /// <returns></returns>
    protected abstract AISettings OnInitializeAISettings();

    /// <summary>
    /// [Must Implement] Handles external invocation that requests the ghost to lose the current player target.
    /// Your implementation doesn't need to actually lose the player, it all depends on the AI's design so be creative!
    /// </summary>
    protected abstract void OnLosePlayerTargetRequested();

    /// <summary>
    /// [Must Implement] Handles door-related logic during AI updates.
    /// This method is called in every pre-tick when the AI encounters a door
    /// that is along the path to its current target. The AI is expected to process
    /// or interact with the door as needed (e.g., opening, closing, bypassing).
    /// </summary>
    /// <param name="obstacleDoor">The door that need to be handled.</param>
    protected abstract void OnAIDoorObstacleTick(Door obstacleDoor);

    /// <summary>
    /// [Must Call Base] The main function executed at every TICK_INTERVAL. This handles AI logic and 
    /// determines the behavior for the current state. This also calls your defined state tick function
    /// following naming convention of void OnStateTickSTATENAME()
    /// </summary>
    [Server]
    protected virtual void OnAITickServer()
    {
        OnAIPreTickServer();

        // Process movement state
        switch (mMovementState)
        {
            // No movement = AI agent would be stopped and would not move to destination
            case MovementState.NoMovement:
                mAgent.isStopped = true;
                break;

            // Update destination and start moving if player target is set, else stop agent
            case MovementState.MoveTowardsPlayerTarget:
                if (mPlayerTarget)
                {
                    mDestination = mPlayerTarget.transform.position;
                    mAgent.isStopped = false;
                    mAgent.destination = mDestination;
                }
                else
                    mAgent.isStopped = true;
                break;

            // Update destination and start moving if waypoint target is set, else stop agent
            case MovementState.MoveTowardsWaypointTarget:
                if (mWaypointTarget)
                {
                    mDestination = mWaypointTarget.transform.position;
                    mAgent.isStopped = false;
                    mAgent.destination = mDestination;
                }
                else
                    mAgent.isStopped = true;
                break;
        }

        InvokeSafe($"{FUNC_PREFIX_ON_STATE_TICK}{mCurrentAIState}");
        OnAIPostTickServer();
    }

    /// <summary>
    /// [Must Call Base] This is called before OnAITickServer. Can be used for validation logic such as checking if Player target
    /// has become null (disconnected). This way you don't have to do null check inside each individual state.
    /// <para>Note: Base Pre-Tick checks for Door obstacle based on setting's TargetReachedDistance and invokes OnAIDoorObstacleTick() if found Door needing to be handled.</para>
    /// </summary>
    [Server]
    protected virtual void OnAIPreTickServer()
    {
        // Validate door
        Door obstacleDoor = GetDoorObstacle(mAISetting.TargetReachedDistance);
        if (obstacleDoor)
            OnAIDoorObstacleTick(obstacleDoor);
    }

    /// <summary>
    /// This is called after OnAITickServer.
    /// </summary>
    /// <param name="oldState"></param>
    /// <param name="newState"></param>
    [Server]
    protected virtual void OnAIPostTickServer() { }

    /// <summary>
    /// This is called whenever the AI changes to a new state. You can put logic that triggers once when state change happens here.
    /// Base function prints out debug if DEBUG_LOG_ON is TRUE;
    /// </summary>
    /// <param name="oldState">The old state</param>
    /// <param name="newState">The new state</param>
    [Server]
    protected virtual void OnAIStateChangedServer(TState oldState, TState newState)
    {
        LogDebug($"Changed state from {oldState} to {newState}");
    }

    /// <summary>
    /// Use to change the state of the AI, which will handle resetting the state timer and prevents changing to the same state.
    /// Note: current state is represented as an integer which you can switch by passing in the enum as well
    /// </summary>
    /// <param name="newState"></param>
    [Server]
    protected virtual void ChangeAIStateServer(TState newState)
    {
        if (mCurrentAIState.Equals(newState)) return;

        mCurrentStateTimer = 0;
        TState oldStateCache = mCurrentAIState;
        mCurrentAIState = newState;
        string onEnterMethodName = $"{FUNC_PREFIX_ON_STATE_ENTER}{newState}";
        string onExitMethodName = $"{FUNC_PREFIX_ON_STATE_EXIT}{oldStateCache}";
        InvokeSafe(onExitMethodName);
        InvokeSafe(onEnterMethodName);
        OnAIStateChangedServer(oldStateCache, mCurrentAIState);
    }

    /// <summary>
    /// <para>[Must Call Base] Handles visibility changes for AI when switching between time frames (e.g., past/present).</para>
    /// 
    /// This function is local and only affects the client-side. It enables or disables 
    /// AI components such as MeshRenderer, AudioSource, and Collider based on visibility status. 
    /// Non-networked, meaning changes apply only locally.
    /// 
    /// <para>If visibility is disabled, the function caches originally inactive components 
    /// and reverts their state upon re-enabling visibility.</para>
    /// </summary>
    /// <param name="isVisible">Determines whether to enable or disable visibility-related components.</param>
    public virtual void HandleAIVisibilityChangeClient(bool isVisible)
    {
        // Clear caches if hiding, to keep track of originally disabled components.
        if (!isVisible)
        {
            mOriginallyInactiveRendererCache.Clear();
            mOriginallyInactiveAudioSourceCache.Clear();
            mOriginallyInactiveColliderCache.Clear();
        }

        // Manage Renderer components visibility
        foreach (Renderer renderer in GetComponentsInChildren<Renderer>())
        {
            if (!isVisible && !renderer.enabled)
                // Cache components that were originally disabled
                mOriginallyInactiveRendererCache.Add(renderer);

            renderer.enabled = isVisible;
        }

        // Manage AudioSource components visibility
        foreach (AudioSource audio in GetComponentsInChildren<AudioSource>())
        {
            if (!isVisible && !audio.enabled)
                mOriginallyInactiveAudioSourceCache.Add(audio);

            audio.enabled = isVisible;
        }

        // Manage Collider components visibility
        foreach (var collider in GetComponentsInChildren<Collider>())
        {
            if (!isVisible && !collider.enabled)
                mOriginallyInactiveColliderCache.Add(collider);

            collider.enabled = isVisible;
        }

        // Re-disable originally inactive components if making everything visible
        if (isVisible)
        {
            foreach (Renderer renderer in mOriginallyInactiveRendererCache)
                renderer.enabled = false;

            foreach (AudioSource audio in mOriginallyInactiveAudioSourceCache)
                audio.enabled = false;

            foreach (Collider collider in mOriginallyInactiveColliderCache)
                collider.enabled = false;
        }
    }
    #endregion

    #region EXPOSED AI API

    /// <summary>
    /// Rotates the AI to face a target Transform smoothly.
    /// </summary>
    /// <param name="target">The target Transform to face.</param>
    /// <param name="rotationSpeed">The speed at which the AI should rotate towards the target, in degrees per second.</param>
    protected void FaceTarget(Transform target, float rotationSpeed = 5f)
    {
        // Calculate the direction to the target
        Vector3 direction = (target.position - transform.position).normalized;
        direction.y = 0; // Ignore vertical difference to rotate only on the Y-axis

        // Determine the target rotation
        Quaternion targetRotation = Quaternion.LookRotation(direction);

        // Smoothly rotate towards the target
        transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation, rotationSpeed * Time.deltaTime);
    }

    /// <summary>
    /// Sets the NavMeshAgent's speed. This does not affect animation speed.
    /// </summary>
    /// <param name="speed"></param>
    protected void SetSpeed(float speed)
    {
        if (Mathf.Approximately(mAgent.maxSpeed, speed)) return;
        mAgent.maxSpeed = speed;
    }
    /// <summary>
    /// Plays a specified animation clip, if the same clip is already playing it will keep playing.
    /// </summary>
    /// <param name="clip"></param>
    protected void PlayAnimation(AnimationClip clip)
    {
        GetAnimancer().Play(clip);
    }

    /// <summary>
    /// Plays a specified animation clip and invokes callback after animation finishes playing.
    /// </summary>
    /// <param name="onEndCallback">The function to invoke after animation finishes.</param>
    /// <param name="clip"></param>
    protected void PlayAnimation(AnimationClip clip, Action onEndCallback)
    {
        PlayAnimation(clip, onEndCallback, null, 0);
    }

    /// <summary>
    /// Plays a specified animation clip and invokes a callback function when the animation reaches a specific percentage of its playback.
    /// </summary>
    /// <param name="animationClip">The animation clip to play.</param>
    /// <param name="onEndCallback">The function to invoke after animation finishes.</param>
    /// <param name="funcToTrigger">The function to invoke during the animation playback.</param>
    /// <param name="triggerAtPercentage">The percentage (0 to 1) of the animation's playback at which the callback is invoked.</param>
    protected void PlayAnimation(AnimationClip animationClip, Action onEndCallback, Action funcToTrigger, float triggerAtPercentage)
    {
        AnimancerState state = GetAnimancer().Play(animationClip);

        if (state.Events(this, out AnimancerEvent.Sequence events))
        {
            if (funcToTrigger != null)
                events.Add(triggerAtPercentage, funcToTrigger);

            if (onEndCallback != null)
                events.OnEnd = onEndCallback;
            return;
        }
    }

    /// <summary>
    /// Returns true if door is successfully opened, false otherwise (door is already opened/not obstacle door).
    /// </summary>
    /// <param name="door"></param>
    /// <returns></returns>
    protected bool TryOpenDoor(Door door)
    {
        if (IsObstacleDoor(door))
        {
            // Guard to prevent AI from closing the door on it-self
            door.ForceOpen();
            return true;
        }

        return false;
    }

    /// <summary>
    /// Returns the closest closed Door that's an obstacle (needs to be opened) and is along the path to the current target.
    /// <para>Note: This does not return Hiding Spot doors.</para>
    /// </summary>
    /// <param name="reachedDistance">Distance within which the door is considered reachable.</param>
    /// <returns>The closest obstacle door or null if no such door exists.</returns>
    protected Door GetDoorObstacle(float reachedDistance)
    {
        // Get all potential door colliders within vision distance
        Collider[] doorColliders = Physics.OverlapSphere(transform.position, mAISetting.VisionDistance, mDoorMask);
        Door closestObstacleDoor = null;
        float closestDistance = float.MaxValue;

        foreach (Collider doorCol in doorColliders)
        {
            Vector3 doorPosition = doorCol.transform.position;

            // Check visibility, distance, and ensure the door is on the path to the target
            if (CanSeeTarget(doorPosition) &&
            IsWithinDistance(doorPosition, reachedDistance))
            {
                Door door = doorCol.GetComponentInParent<Door>();

                // Check if the door is a valid obstacle
                if (IsObstacleDoor(door))
                {
                    float distanceToDoor = Vector3.Distance(transform.position, doorPosition);
                    if (distanceToDoor < closestDistance)
                    {
                        closestObstacleDoor = door;
                        closestDistance = distanceToDoor;
                    }
                }
            }
        }

        return closestObstacleDoor;
    }

    /// <summary>
    /// If player target is not valid, drops the player target by SetPlayerTarget(null).
    /// </summary>
    /// <returns>Returns true if the current player target is set, alive, and in the same time frame as AI, false otherwise.</returns>
    protected bool ValidatePlayerTarget()
    {
        Player player = GetPlayerTarget();
        bool validTarget = player && player.mIsAlive && (BaseTimeframeManager.Instance.GetIsPastLocal() == player.isPlayerInPast);
        if (!validTarget)
            SetPlayerTarget(null);

        return validTarget;
    }

    /// <summary>
    /// Attempts to attack all currently seen (visible) players in the specified range by radius.
    /// </summary>
    /// <param name="radius"></param>
    /// <returns>true if at least one player in radius is attacked, false if all players in radius failed to be attacked (not visible to AI). </returns>
    protected bool TryAttackAllPlayersInRadius(float radius, int damageAmount)
    {
        // Use override of FindAllVisiblePlayers
        // Do not count alert distance as automatically visible
        List<Player> visiblePlayers = FindAllVisiblePlayers(false, radius);
        foreach (Player player in visiblePlayers)
            player.OnSanityHit(this, damageAmount);

        return visiblePlayers.Count > 0;
    }

    /// <summary>
    /// Attempts to attack the specified player with specified damage. Attack succeeds if player can be seen in the specified attack range.
    /// </summary>
    /// <param name="player"></param>
    /// <param name="damageAmount"></param>
    /// <returns>true if attack is successful, false otherwise.</returns>
    protected bool TryAttackPlayer(Player player, float attackRange, int damageAmount)
    {
        if (CanSeePlayer(player, attackRange))
        {
            player.OnSanityHit(this, damageAmount);
            return true;
        }

        return false;
    }

    /// <summary>
    /// Invoke to spawn effects like footprint and hand print at AI's current location.
    /// Defined effect names are const such as EFFECT_FOOT_PRINT and EFFECT_HAND_PRINT.
    /// <para>Note: Spawning EFFECT_FOOT_PRINT would automatically alternate left/right steps.</para>
    /// </summary>
    /// <param name="effectName"></param>
    protected void SpawnEffectServer(string effectName)
    {
        if (effectName == EFFECT_FOOT_PRINT)
            mPreviousFootstepFlipX = !mPreviousFootstepFlipX;
        BaseEffectsManager.Instance.SpawnEffectServer(effectName, transform.position + new Vector3(0, mAgent.height / 2, 0), transform.forward, effectName == EFFECT_FOOT_PRINT ? mPreviousFootstepFlipX : AIMath.Decide2());
    }

    /// <summary>
    /// Finds the closest player that can be seen by AI based on defined settings for vision and alert distance (always visible to AI if alive and within this distance and not hiding).
    /// <para>This is a simplified function wrapping FindClosest() passing in result from FindAllVisiblePlayers()</para>
    /// <para>Note: This uses CanSeePlayer(), thus player in a different time frame before LIMEN Break or dead is not considered.</para>
    /// </summary>
    /// <param name="includeAlertDistance">set to true to automatically treat player within alert distance counts as being seen/visible.</param>
    /// <param name="overrideVisionDistance">set to use for vision distance instead of using the vision distance defined in current AI settings.</param>
    /// <returns></returns>
    protected Player FindClosestVisiblePlayer(bool includeAlertDistance, float overrideVisionDistance = float.NegativeInfinity)
    {
        return FindClosest(FindAllVisiblePlayers(includeAlertDistance, overrideVisionDistance));
    }

    /// <summary>
    /// Finds all players that can be seen by AI based on defined settings for vision and alert distance (always visible to AI if alive and within this distance and not hiding).
    /// <para>Note: This uses CanSeePlayer(), thus player in a different time frame before LIMEN Break is not considered.</para>
    /// </summary>
    /// <param name="includeAlertDistance">set to true to automatically treat player within alert distance counts as being seen/visible.</param>
    /// <param name="overrideVisionDistance">set to use for vision distance instead of using the vision distance defined in current AI settings.</param>
    /// <returns>A list containing visible players, empty if found none.</returns>
    protected List<Player> FindAllVisiblePlayers(bool includeAlertDistance, float overrideVisionDistance = float.NegativeInfinity)
    {
        List<Player> results = new List<Player>();
        Collider[] targetsInViewRadius = Physics.OverlapSphere(mHeadBone.position, overrideVisionDistance >= 0 ? overrideVisionDistance : mAISetting.VisionDistance, mPlayerMask);
        for (int i = 0; i < targetsInViewRadius.Length; i++)
        {
            Player targetPlayerComp = targetsInViewRadius[i].transform.root.GetComponent<Player>();
            if (results.Contains(targetPlayerComp)) continue;
            // Check if player in alert distance is not currently in a hiding spot, as AI could be outside one and wrongly add that player as a visible one
            if ((includeAlertDistance && IsWithinDistance(targetsInViewRadius[i].transform.position, mAISetting.AlertDistance) && targetPlayerComp.currentHidingSpot == null && targetPlayerComp.mIsAlive) || CanSeePlayer(targetPlayerComp))
                results.Add(targetPlayerComp);
        }

        return results;
    }

    /// <summary>
    /// Simplified function to check if a player can be seen by AI based on the initialized AI settings
    /// (vision distance, angle, etc). This also checks for target blockage (are there obstacles in between).
    /// <para>Note: The difference between this vs CanSeeTarget(Vector3 target) is that this checks for time frame and if player is alive in addition. 
    /// A player that's in a different time frame than ghost, before LIMEN Break, or is dead, is not visible to ghost.</para>
    /// </summary>
    /// <param name="target">The target position to check for</param>
    /// <param name="overrideVisionDistance">set to use for vision distance instead of using the vision distance defined in current AI settings.</param>
    /// <returns>True if target can be seen (in fov and not blocked by obstacles), false otherwise.</returns>
    protected virtual bool CanSeePlayer(Player target, float overrideVisionDistance = float.NegativeInfinity)
    {
        // If LIMEN Break has not occurred and player is in a different time frame than AI, then it's not visible so return false
        if ((!BaseTimeframeManager.Instance.GetHasLimenBreakOccuredLocal() && (BaseTimeframeManager.Instance.GetIsPastLocal() != target.isPlayerInPast)) || !target.mIsAlive) return false;
        return IsTargetVisible(target.transform.position, overrideVisionDistance >= 0 ? overrideVisionDistance : mAISetting.VisionDistance, mAISetting.VisionAngle, out Transform _discard);
    }

    /// <summary>
    /// Simplified function to check if a target can be seen by AI based on the initialized AI settings
    /// (vision distance, angle, etc). This also checks for target blockage (are there obstacles in between).
    /// </summary>
    /// <param name="target">The target position to check for</param>
    /// <param name="overrideVisionDistance">set to use for vision distance instead of using the vision distance defined in current AI settings.</param>
    /// <returns>True if target can be seen (in fov and not blocked by obstacles), false otherwise.</returns>
    protected virtual bool CanSeeTarget(Vector3 target, float overrideVisionDistance = float.NegativeInfinity)
    {
        return IsTargetVisible(target, overrideVisionDistance >= 0 ? overrideVisionDistance : mAISetting.VisionDistance, mAISetting.VisionAngle, out Transform _discard);
    }

    /// <summary>
    /// Checks if a target can be seen by AI with specified vision distance and vision angle.
    /// <para>Casts multiple rays around the target to check for partial blockage, offsetting up/down and left/right for better accuracy.
    /// The number of rays set in NUM_VISION_RAYS determines how the FOV is sampled; more rays increases accuracy but reduces performance.</para>
    /// </summary>
    /// <param name="target">The target position to check visibility for.</param>
    /// <param name="visionDistance">The maximum distance the AI can see the target.</param>
    /// <param name="visionAngle">The horizontal vision angle of the AI, in degrees, measured from the forward direction.</param>
    /// <param name="obstacle">The obstacle blocking the view, if any.</param>
    /// <returns>True if the target is in fov and is visible (not blocked by any obstacles), false otherwise.</returns>
    protected virtual bool IsTargetVisible(Vector3 target, float visionDistance, float visionAngle, out Transform obstacle)
    {
        obstacle = null;

        // Check frustum first as it's less expensive
        if (!IsTargetInFieldOfView(target, visionDistance, visionAngle))
            return false;

        Vector3 toTarget = target - transform.position;

        // Calculate dynamic step sizes based on the number of rays and vision angle
        float horizontalStep = visionAngle / (NUM_VISION_RAYS - 1);
        float verticalViewAngle = 60f;  // Assume a fixed vertical FOV of 60
        float verticalStep = verticalViewAngle / (NUM_VISION_RAYS - 1);

        // Calculate the normalized direction to the target and the distance
        Vector3 initialRayDirection = toTarget.normalized;
        float distanceToTarget = toTarget.magnitude;

        // Iterate through the ray offsets to check for obstructions
        for (int h = 0; h < NUM_VISION_RAYS; h++)
        {
            for (int v = 0; v < NUM_VISION_RAYS; v++)
            {
                // Calculate the horizontal and vertical offsets based on ray counts
                float horizontalOffset = (NUM_VISION_RAYS > 1) ? (-visionAngle * 0.5f + h * horizontalStep) : 0.0f;
                float verticalOffset = (NUM_VISION_RAYS > 1) ? (-verticalViewAngle * 0.5f + v * verticalStep) : 0.0f;

                // Apply the offset to the initial ray direction
                Vector3 rayDirection = Quaternion.Euler(verticalOffset, horizontalOffset, 0) * initialRayDirection;

                // Perform the raycast to check for obstacles
                if (!Physics.Raycast(mHeadBone.position, rayDirection, out RaycastHit hitInfo, distanceToTarget, mObstacleMask))
                {
                    obstacle = null;
                    return true; // No obstacle found, target is visible
                }

                // Update the obstacle if a hit occurs
                obstacle = hitInfo.collider.transform;
            }
        }

        // All rays are blocked, target is not visible
        return false;
    }

    /// <summary>
    /// Checks if a target is within FOV, considering only 2D space (ignoring Y-axis).
    /// The check is based on vision distance and vision angle specified into the function.
    /// <para>Note: This does not check if the target can be seen (blocked by obstacles); it only checks the FOV range and angle.</para>
    /// </summary>
    /// <param name="target">The position of the target to check.</param>
    /// <param name="visionDistance">The maximum distance the AI can see the target.</param>
    /// <param name="visionAngle">The vision angle of the AI, in degrees, measured from the forward direction.</param>
    /// <returns>True if the target is within the AI's vision distance and angle, otherwise false.</returns>
    protected bool IsTargetInFieldOfView(Vector3 target, float visionDistance, float visionAngle)
    {
        // Check if the target is within the vision distance (2D)
        if (Distance2D(mHeadBone.position, target) > visionDistance)
            return false;

        // Calculate direction to the target and ignore the Y-axis
        Vector3 direction = target - mHeadBone.position;
        direction.y = 0;
        direction.Normalize(); // Normalize the direction for accurate angle checking

        // Get forward direction of the head, ignoring the Y-axis
        Vector3 forward = mHeadBone.forward;
        forward.y = 0;
        forward.Normalize();

        // Check if the angle between forward direction and target direction is within the allowed vision angle
        return Vector3.Angle(forward, direction) <= visionAngle / 2;
    }
    #endregion

    #region EXPOSED AI CALLBACKS
    /// <summary>
    /// [Invokes on Server Only] Callback when a HostMachine is attacked, override to add response logic.
    /// </summary>
    /// <param name="hostMachine">The HostMachine that's being attacked.</param>
    protected virtual void OnHostMachineAttackedServer(HostMachine hostMachine)
    {

    }

    /// <summary>
    /// [Invokes on Server Only] Callback when a HostMachine is destroyed, override to add response logic.
    /// </summary>
    /// <param name="hostMachine"></param>
    protected virtual void OnHostMachineDestroyedServer(HostMachine hostMachine)
    {

    }
    #endregion

    #region EXPOSED HELPER API

    /// <summary>
    /// Invokes a method only if it exists.
    /// </summary>
    /// <param name="methodName"></param>
    protected void InvokeSafe(string methodName)
    {
        // Check if the method has already been cached
        if (mStateInvokeHasMethodDictionary.TryGetValue(methodName, out bool hasMethod))
        {
            if (hasMethod)
                Invoke(methodName, 0);
        }
        else
        {
            // Find all matching methods in the class hierarchy
            var methods = GetType().GetMethods(BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public)
                                   .Where(m => m.Name == methodName)
                                   .ToList();

            // Throw an error if more than one method with the same name exists
            if (methods.Count > 1)
            {
                throw new AmbiguousMatchException($"Multiple methods named '{methodName}' found in type hierarchy.");
            }

            // Cache the result and recursively try invoking if a method exists
            bool methodExists = methods.Count == 1;
            mStateInvokeHasMethodDictionary[methodName] = methodExists;

            if (methodExists)
                InvokeSafe(methodName);
        }
    }

    /// <summary>
    /// Returns true if the specified door is currently an obstacle (closed door) on AI's path to target that need to be handled.
    /// </summary>
    /// <param name="door"></param>
    /// <returns></returns>
    protected bool IsObstacleDoor(Door door)
    {
        return door && !door.isHidingSpotDoor && (!door.isOpened || door.IsDoorMoving()) && IsPathObstacle(door.transform.position);
    }

    /// <summary>
    /// Checks if a position is on the current NavMeshAgent path, thus an obstacle on AI's path to target.
    /// <para>Note: This doesn't mean that the position has NavMeshObstacle resulting in path failure or partial,
    /// it's based on the assumption that passed in position is a potential obstacle such as Door, and this is to verify that it's
    /// significant to the path.</para>
    /// </summary>
    /// <param name="position">The position to check.</param>
    /// <param name="agent">The NavMeshAgent whose path to check against.</param>
    /// <param name="threshold">The maximum distance from the path to consider the position "on the path".</param>
    /// <returns>True if AI has path and the position is on the path, hence is an obstacle along AI's path to target; otherwise, false.</returns>
    public bool IsPathObstacle(Vector3 position, float threshold = 1.5f)
    {
        if (!mAgent.hasPath)
            return false;

        List<Vector3> buffer = new List<Vector3>();
        mAgent.GetRemainingPath(buffer, out bool stale);

        // Check each segment of the path
        for (int i = 0; i < buffer.Count - 1; i++)
        {
            Vector3 segmentStart = buffer[i];
            Vector3 segmentEnd = buffer[i + 1];

            // Check if the position is near the segment
            if (IsPointNearLineSegment(position, segmentStart, segmentEnd, threshold))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Calculates the 2D distance between two Vector3 points, ignoring the Y-axis.
    /// </summary>
    /// <param name="a">The first Vector3 point.</param>
    /// <param name="b">The second Vector3 point.</param>
    /// <returns>The 2D distance between the two points, ignoring Y.</returns>
    protected float Distance2D(Vector3 a, Vector3 b)
    {
        // Convert to Vector2, ignoring Y-axis
        Vector2 a2D = new Vector2(a.x, a.z);
        Vector2 b2D = new Vector2(b.x, b.z);

        return Vector2.Distance(a2D, b2D);
    }

    /// <summary>
    /// Simplified function to check if a target is within the specified distance using Vector3.Distance(transform.position, target).
    /// </summary>
    /// <param name="target"></param>
    /// <param name="distance"></param>
    /// <returns>True if target is within distance using Vector3, false otherwise.</returns>
    protected bool IsWithinDistance(Vector3 target, float distance)
    {
        return Vector3.Distance(transform.position, target) <= distance;
    }

    /// <summary>
    /// Finds the closest MonoBehaviour derived component in a list to the current AI.
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="list"></param>
    /// <returns>The closest MonoBehaviour derived component, null if list is empty or does not contain any non-null entries.</returns>
    protected T FindClosest<T>(List<T> list) where T : MonoBehaviour
    {
        if (list == null || list.Count == 0) return null;

        T closest = null;
        float closestDistanceSqr = Mathf.Infinity;

        // Iterate through the list to find the closest T
        foreach (T item in list)
        {
            if (item == null) continue; // Skip null items
            float distanceSqr = (item.transform.position - transform.position).sqrMagnitude;

            if (distanceSqr < closestDistanceSqr)
            {
                closestDistanceSqr = distanceSqr;
                closest = item;
            }
        }

        return closest;
    }
    #endregion

    #region DEBUG

    /// <summary>
    /// Displays debug information about the AI in the form of a 3D text mesh.
    /// This works at runtime in standalone builds.
    /// </summary>
    protected void DisplayDebugInfo()
    {
        if (!isServer) return;
        // Create or find the debug text object
        string debugTextObjectName = "AIDebugText";
        Transform existingDebugText = transform.Find(debugTextObjectName);
        TextMesh debugTextMesh;

        if (existingDebugText == null)
        {
            GameObject debugTextObject = new GameObject(debugTextObjectName);
            debugTextObject.transform.SetParent(transform);
            debugTextObject.transform.localPosition = new Vector3(0, 2, 0); // Position above the AI
            debugTextMesh = debugTextObject.AddComponent<TextMesh>();
            debugTextMesh.characterSize = 0.1f;
            debugTextMesh.anchor = TextAnchor.MiddleCenter;
            debugTextMesh.alignment = TextAlignment.Center;
            debugTextMesh.color = Color.green; // Set text color to green
            debugTextMesh.fontStyle = FontStyle.Normal;
        }
        else
        {
            debugTextMesh = existingDebugText.GetComponent<TextMesh>();
        }

        // Gather debug information
        string navMeshState = mAgent.isStopped ? "Stopped" : "Moving";
        float speed = mAgent.velocity.magnitude;
        string playerTarget = mPlayerTarget != null ? mPlayerTarget.name : "None";
        string waypointTarget = mWaypointTarget != null ? mWaypointTarget.name : "None";
        string currentState = mCurrentAIState.ToString();

        // Set the text mesh content
        debugTextMesh.text = $"AI Debug Info:\n\n" +
                             $"AI State: {currentState}\n" +
                             $"Movement State: {mMovementState}\n" +
                             $"NavMesh State: {navMeshState}\n" +
                             $"Speed: {speed:F2}\n" +
                             $"Player Target: {playerTarget}\n" +
                             $"Waypoint Target: {waypointTarget}\n";

        // Make sure the text always faces the camera
        if (Player.LocalPlayer != null)
        {
            debugTextMesh.transform.rotation = Quaternion.LookRotation(debugTextMesh.transform.position - Player.LocalPlayer.mainCamera.transform.position);
        }
    }

    #endregion
}
