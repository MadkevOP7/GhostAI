// Copyright © 2024 by MADKEV Studio, all rights reserved

using System;
using System.Collections;
using Animancer;
using UnityEngine;

/// <summary>
/// Implementation for Eye Man ghost character.
/// </summary>
public class EyeMan : BaseGhostAI<EyeMan.AIStates>
{
    private const float WALK_SPEED = 3f;
    private const float RUN_SPEED = 6.8f;
    private const int CHOKE_ATTACK_AMOUNT = 100;
    private const int LONG_ATTACK_AMOUNT = 68;
    private const float ATTACK_RANGE = 2.86f;
    private const float CHOKE_ATTACK_RANGE = 1.26f;
    [Header("Setup")]
    public Transform mHandTransform;
    public AnimationClip mWalkClip;
    public AnimationClip mRunClip;
    public AnimationClip mStandingClip;
    public AnimationClip mChokeAttackClip;
    public AnimationClip mLongAttackClip;

    private float mDoorWaitTime;
    private Door mObstacleDoor;
    protected override AISettings OnInitializeAISettings()
    {
        AISettings setting = new AISettings();
        setting.StartingState = AIStates.Wondering;
        setting.AlertDistance = 5;
        setting.VisionAngle = 145;
        setting.VisionDistance = 10;
        return setting;
    }

    protected override void OnLosePlayerTargetRequested()
    {
        // Get difficulty multiplier to see if we should lose player
        // More difficult has higher percentage, so here we need to 1 - percent to calculate percentage chance of losing player
        if (AIMath.Decide2(LIMENDefine.GetAIProbabilityPercent(1 - GameManager.Instance.GetDifficulty())))
        {
            // Lose player: set state to wondering. We don't clear the current player target,
            // but if player target is no longer visible, AI won't keep chasing
            ChangeAIStateServer(AIStates.Wondering);
        }
    }

    public enum AIStates
    {
        Wondering, // This will be the default state since it's the first enum
        Chase,
        Attack,
        HandleDoor,
    }

    protected override void OnAIDoorObstacleTick(Door obstacleDoor)
    {
        mObstacleDoor = obstacleDoor;
        ChangeAIStateServer(AIStates.HandleDoor);
    }

    // TICKING //
    protected override void OnAIPreTickServer()
    {
        base.OnAIPreTickServer();
        // Validate if player target becomes null (Player disconnected) before invoking base
        // So we don't have to null check or validate if player is already killed in every ticking state later
        // Pre tick validate
        if (!ValidatePlayerTarget() && (GetCurrentState() == AIStates.Chase || GetCurrentState() == AIStates.Attack))
            ChangeAIStateServer(AIStates.Wondering);
    }

    // State - HandleDoor
    protected void OnStateEnterHandleDoor()
    {
        SetMovementState(BaseGhostAI<AIStates>.MovementState.NoMovement);
        mDoorWaitTime = UnityEngine.Random.Range(1.0f, 3.0f);
    }

    protected void OnStateTickHandleDoor()
    {
        PlayAnimation(mStandingClip);
        if (mObstacleDoor)
            FaceTarget(mObstacleDoor.transform);
        // Wait a random amount before proceeding to open the door or if door is no longer closed
        if (!IsObstacleDoor(mObstacleDoor) || GetCurrentStateTimer() >= mDoorWaitTime)
        {
            // Leave a hand-print on the door after opening it
            if (TryOpenDoor(mObstacleDoor))
                SpawnEffectServer(EFFECT_HAND_PRINT);

            // Change to attack, AI pre-tick would fallback to wondering if does not have valid player target.
            ChangeAIStateServer(AIStates.Chase);
        }
    }

    // State - Wondering //
    protected void OnStateEnterWondering()
    {
        SetMovementState(BaseGhostAI<AIStates>.MovementState.MoveTowardsWaypointTarget);
    }

    protected void OnStateTickWondering()
    {
        PlayAnimation(mWalkClip);
        SetSpeed(WALK_SPEED);
        // Patrol waypoint
        if (GetWaypointTarget() == null || IsWithinDistance(GetWaypointTarget().transform.position, 2f))
            SetWaypointTarget(GlobalWaypoint.Instance.GetNextWaypoint(GetWaypointTarget()));

        // Search for player
        Player foundPlayer = FindClosestVisiblePlayer(true);
        if (foundPlayer)
        {
            SetPlayerTarget(foundPlayer);
            ChangeAIStateServer(AIStates.Chase);
        }
    }

    // State - Chase //
    protected void OnStateEnterChase()
    {
        SetMovementState(BaseGhostAI<AIStates>.MovementState.MoveTowardsPlayerTarget);
    }
    protected void OnStateTickChase()
    {
        GetPlayerTarget().NotifyChase(this);
        PlayAnimation(mRunClip);
        SetSpeed(RUN_SPEED);
        // Thanks to our player target check in OnAITickServer, we don't have to null check here for player target
        if (IsWithinDistance(GetPlayerTarget().transform.position, ATTACK_RANGE))
            ChangeAIStateServer(AIStates.Attack);
    }

    protected void OnStateEnterAttack()
    {
        // Decide which attack to use, choke attack or ranged attack
        // If player is still very close and visible to AI, use choke attack. Else, use long range attack that deals regional damage to other players in radius as well.
        bool useChokeAttack = CanSeePlayer(GetPlayerTarget()) && IsWithinDistance(GetPlayerTarget().transform.position, CHOKE_ATTACK_RANGE);
        if (useChokeAttack)
        {
            GetPlayerTarget().ServerPinPlayerToTransform(HumanBodyBones.Head, mHandTransform, mChokeAttackClip.length);
            PlayAnimation(mChokeAttackClip, OnAttackEnded, () => { DoAttack(true); }, 0.52f);
        }
        else
            PlayAnimation(mLongAttackClip, OnAttackEnded, () => { DoAttack(false); }, 0.426f);
    }

    protected void OnStateTickAttack()
    {
        GetPlayerTarget().NotifyChase(this);
        SetMovementState(BaseGhostAI<AIStates>.MovementState.NoMovement);
        FaceTarget(GetPlayerTarget().transform);
    }

    // Invoked after attack animation has finished playing.
    private void OnAttackEnded()
    {
        // Reset state to chase, pre-tick will handle fallback to wonder if no valid player target is set
        ChangeAIStateServer(AIStates.Chase);
    }

    // Perform the actual attack and deal damage to players
    private void DoAttack(bool isChokeAttack)
    {
        if (isChokeAttack)
            GetPlayerTarget().OnSanityHit(this, CHOKE_ATTACK_AMOUNT);
        else
            TryAttackAllPlayersInRadius(ATTACK_RANGE, LONG_ATTACK_AMOUNT);
    }
}
