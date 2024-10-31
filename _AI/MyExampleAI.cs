using UnityEngine;

public class MyExampleAI : BaseGhostAI<MyExampleAI.AIStates>
{
    protected override AISettings OnInitializeAISettings()
    {
        AISettings setting = new AISettings();
        setting.StartingState = AIStates.Wondering;
        return setting;
    }

    public enum AIStates
    {
        Wondering, // This will be the default state since it's the first enum
        Chase,
        Attack
    }

    // TICKING //
    protected override void OnAIPreTickServer()
    {
        base.OnAIPreTickServer();
        Debug.Log("Pre-Tick!");
        // Validate if player target becomes null (Player disconnected) before invoking base
        // So we don't have to null check or validate if player is already killed in every ticking state later
        // Pre tick validate
        if (GetPlayerTarget() == null || !GetPlayerTarget().is_alive)
            ChangeAIStateServer(AIStates.Wondering);
    }
    protected override void OnAITickServer()
    {
        Debug.Log("AI Tick!");
        // Todo: update position of self to a manager of some sorts...
        // Do it here for things that are run in multiple states, so you don't have to
        // copy paste the same code to all OnStateTick[StateName] functions.

        // INVOKE BASE -- Must include this!!
        base.OnAITickServer();
    }
    protected override void OnAIPostTickServer()
    {
        base.OnAIPostTickServer();
        Debug.Log("AI Finished a Tick!");
    }

    // State - Wondering //
    protected void OnStateEnterWondering()
    {
        Debug.Log("Enter Wondering State: AI will move to current Waypoint target if it's set");
        SetMovementState(BaseGhostAI<AIStates>.MovementState.MoveTowardsWaypointTarget);
    }
    protected void OnStateTickWondering()
    {
        Debug.Log("Ticking Wondering State: Patrolling waypoint and looking for player target");
        // Patrol waypoint
        if (GetWaypointTarget() == null || IsWithinDistance(GetWaypointTarget().transform.position, 2f))
            SetWaypointTarget(GlobalWaypoint.Instance.GetNextWaypoint(GetWaypointTarget()));

        // Search for player
        Player foundPlayer = FindClosestVisiblePlayer();
        if (foundPlayer)
        {
            SetPlayerTarget(foundPlayer);
            ChangeAIStateServer(AIStates.Chase);
        }
    }
    protected void OnStateExitWondering()
    {
        Debug.Log("Exit Wondering state");
    }

    // State - Chase //
    protected void OnStateEnterChase()
    {
        Debug.Log("Enter Chase State: AI will move to current Player target if it's set");
        SetMovementState(BaseGhostAI<AIStates>.MovementState.MoveTowardsPlayerTarget);
    }
    protected void OnStateTickChase()
    {
        Debug.Log("Ticking Chase State: Looking for the right time to attack player!");
        float attackDist = 2;
        // Thanks to our player target check in OnAITickServer, we don't have to null check here for player target
        if (IsWithinDistance(GetPlayerTarget().transform.position, attackDist))
            ChangeAIStateServer(AIStates.Attack);
    }

    // State - Attack //
    protected void OnStateEnterAttack()
    {
        // Thanks to our player target check in OnAITickServer, we don't have to null check here for player target too!
        Debug.Log("Enter Attack State: Dealing damage to player!");
        int attackDamage = 68;
        GetPlayerTarget().OnSanityHit(attackDamage);

        // We also don't have to check if player is killed, as we have done that in OnAIPreTickServer().
    }
}
