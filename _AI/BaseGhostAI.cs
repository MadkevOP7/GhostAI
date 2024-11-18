// Copyright © 2024 by MADKEV Studio, all rights reserved

using UnityEngine;
using UnityEngine.AI;
using Mirror;
using System.Collections.Generic;
//[RequireComponent(typeof(GhostIdentity))]
//[RequireComponent(typeof(NetworkTransformReliable))]
//[RequireComponent(typeof(NetworkAnimator))]
[RequireComponent(typeof(NavMeshAgent))]
public abstract class BaseGhostAI<TState> : NetworkBehaviour where TState : System.Enum
{
    public static bool DEBUG_LOG_ON = true;
    #region DATA STRUCTURE
    public class AISettings
    {
        public TState StartingState;
        public int AttackDamage;
        public float WalkSpeed;
        public float RunSpeed;
        public float VisionAngle;
        public float VisionDistance;
        [Tooltip("The distance AI would be alerted if target is within, even if not visible or seen.")]
        public float AlertDistance;
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
    protected LayerMask mObstacleMask = ~(LayerMask.GetMask("Ghost") | LayerMask.GetMask("Player") | LayerMask.GetMask("Ignore Raycast") | LayerMask.GetMask("UI") | LayerMask.GetMask("Post Processing") | LayerMask.GetMask("TransparentFX"));
    protected LayerMask mPlayerMask = LayerMask.GetMask("Player");
    protected LayerMask mDoorMask = LayerMask.GetMask("Door");
    // COMPONENT REFERENCES
    private NavMeshAgent mAgent;

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

    /// <summary>
    /// Returns the current Player target. If mMovingTowardsPlayerTarget is True, AI will move towards this.
    /// </summary>
    /// <returns></returns>
    public Player GetPlayerTarget() { return mPlayerTarget; }
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

    /// <summary>
    /// Returns the current Waypoint target. If mMoveTowardsDestination is True, and mMovingTowardsPlayerTarget AI will move towards this.
    /// </summary>
    /// <returns></returns>
    public Waypoint GetWaypointTarget() { return mWaypointTarget; }
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
    // Effect State
    private bool mPreviousFootstepFlipX = false;
    public float GetCurrentStateTimer() { return mCurrentStateTimer; }
    #region Internal Core - These are not exposed to AI implementations
    /// <summary>
    /// [Must Call Base] Base handles getting component and initializing necessary setup functions
    /// </summary>
    public override void OnStartServer()
    {
        base.OnStartServer();
        mAgent = GetComponent<NavMeshAgent>();
        mAISetting = OnInitializeAISettings();
        AIBrain.Instance.OnHostMachineAttackedCallbackServer += OnHostMachineAttackedServer;
        AIBrain.Instance.OnHostMachineDestroyedCallbackServer += OnHostMachineDestroyedServer;

        // Since defined state starts as default, we need to manually invoke the OnStateEnter function
        // for that default state
        Invoke($"{FUNC_PREFIX_ON_STATE_ENTER}{mCurrentAIState}", 0);
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
        if (mMainLoopTimer >= TICK_INTERVAL)
        {
            mMainLoopTimer = 0;
            OnAITickServer();
        }
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
    #endregion

    #region EXPOSED IMPLEMENTATIONS
    /// <summary>
    /// [Must Implement] Define and return an AISettings struct for customizing your AI.
    /// Initialize other variables declared for your AI implementation here as well.
    /// </summary>
    /// <returns></returns>
    protected abstract AISettings OnInitializeAISettings();

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

        Invoke($"{FUNC_PREFIX_ON_STATE_TICK}{mCurrentAIState}", 0);
        OnAIPostTickServer();
    }

    /// <summary>
    /// This is called before OnAITickServer. Can be used for validation logic such as checking if Player target
    /// has become null (disconnected). This way you don't have to do null check inside each individual state.
    /// </summary>
    [Server]
    protected virtual void OnAIPreTickServer() { }

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
        Invoke(onExitMethodName, 0);
        Invoke(onEnterMethodName, 0);
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
    /// Finds the closest player that can be seen by AI based on defined settings for vision and alert distance (always visible to AI if within this distance and not hiding).
    /// <para>This is a simplified function wrapping FindClosest() passing in result from FindAllVisiblePlayers()</para>
    /// <para>Note: This uses CanSeePlayer(), thus player in a different time frame before LIMEN Break is not considered.</para>
    /// </summary>
    /// <returns></returns>
    protected Player FindClosestVisiblePlayer()
    {
        return FindClosest(FindAllVisiblePlayers());
    }

    /// <summary>
    /// Finds all players that can be seen by AI based on defined settings for vision and alert distance (always visible to AI if within this distance and not hiding).
    /// <para>Note: This uses CanSeePlayer(), thus player in a different time frame before LIMEN Break is not considered.</para>
    /// </summary>
    /// <returns>A list containing visible players, empty if found none.</returns>
    protected List<Player> FindAllVisiblePlayers()
    {
        List<Player> results = new List<Player>();
        Collider[] targetsInViewRadius = Physics.OverlapSphere(mHeadBone.position, mAISetting.VisionDistance, mPlayerMask);
        for (int i = 0; i < targetsInViewRadius.Length; i++)
        {
            Player targetPlayerComp = targetsInViewRadius[i].transform.root.GetComponent<Player>();

            // Check if player in alert distance is not currently in a hiding spot, as AI could be outside one and wrongly add that player as a visible one
            if ((IsWithinDistance(targetsInViewRadius[i].transform.position, mAISetting.AlertDistance) && targetPlayerComp.currentHidingSpot == null) || CanSeePlayer(targetPlayerComp))
                results.Add(targetPlayerComp);
        }

        return results;
    }

    /// <summary>
    /// Simplified function to check if a player can be seen by AI based on the initialized AI settings
    /// (vision distance, angle, etc). This also checks for target blockage (are there obstacles in between).
    /// <para>Note: The difference between this vs CanSeeTarget(Vector3 target) is that this checks for time frame as well. 
    /// A player that's in a different time frame than ghost, before LIMEN Break, is not visible to ghost.</para>
    /// </summary>
    /// <param name="target">The target position to check for</param>
    /// <returns>True if target can be seen (in fov and not blocked by obstacles), false otherwise.</returns>
    protected virtual bool CanSeePlayer(Player target)
    {
        // If LIMEN Break has not occurred and player is in a different time frame than AI, then it's not visible so return false
        if (!BaseTimeframeManager.Instance.GetHasLimenBreakOccuredLocal() && (BaseTimeframeManager.Instance.GetIsPastLocal() != target.isPlayerInPast)) return false;
        return IsTargetVisible(target.transform.position, mAISetting.VisionDistance, mAISetting.VisionAngle, out Transform _discard);
    }

    /// <summary>
    /// Simplified function to check if a target can be seen by AI based on the initialized AI settings
    /// (vision distance, angle, etc). This also checks for target blockage (are there obstacles in between).
    /// </summary>
    /// <param name="target">The target position to check for</param>
    /// <returns>True if target can be seen (in fov and not blocked by obstacles), false otherwise.</returns>
    protected virtual bool CanSeeTarget(Vector3 target)
    {
        return IsTargetVisible(target, mAISetting.VisionDistance, mAISetting.VisionAngle, out Transform _discard);
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
        return Vector3.Angle(forward, direction) <= visionAngle;
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
}
