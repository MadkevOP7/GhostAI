using UnityEngine;
using System.Collections;
using UnityEditor;

[CustomEditor (typeof (GhostAI), true)]
public class AIEditor : Editor {

	void OnSceneGUI() {
		GhostAI ai = (GhostAI)target;
		Handles.color = Color.white;
		Handles.DrawWireArc (ai.transform.position, Vector3.up, Vector3.forward, 360, ai.visionDistance);
		Vector3 viewAngleA = ai.DirFromAngle (-ai.viewAngle / 2, false);
		Vector3 viewAngleB = ai.DirFromAngle (ai.viewAngle / 2, false);

		Handles.DrawLine (ai.transform.position, ai.transform.position + viewAngleA * ai.visionDistance);
		Handles.DrawLine (ai.transform.position, ai.transform.position + viewAngleB * ai.visionDistance);

        if(ai.currentTarget != null)
        {
            Handles.color = Color.blue;
            Handles.DrawLine(ai.transform.position, ai.currentTarget.targetTransform.position);
        }

        if (ai.currentAttackTarget != null)
        {
            Handles.color = Color.red;
            Handles.DrawLine(ai.transform.position, ai.currentAttackTarget.targetTransform.position);
        }
		
        if (ai.agent != null)
        {
            if (ai.agent.pathStatus == UnityEngine.AI.NavMeshPathStatus.PathComplete)
            {
                Handles.color = Color.green;
            }
            else
            {
                Handles.color = Color.red;
            }
            for (int i = 0; i < ai.agent.path.corners.Length - 1; i++)
            {
                Handles.DrawLine(ai.agent.path.corners[i], ai.agent.path.corners[i + 1]);
            }
        }

        DrawAIInfoState(ai);

    }

    void DrawAIInfoState(GhostAI ai)
    {
        GUIStyle style = new GUIStyle();
        style.normal.textColor = Color.green;
        string info = "AI Position: " + ai.transform.position + "\n";
        info += "AI State: " + ai.DAIState.ToString() + "\n";
        info += ai.currentTarget != null && ai.currentTarget.targetTransform != null ? "Target: " + ai.currentTarget.targetTransform.name + " Position: " + ai.currentTarget.targetTransform.position : "Target: null";
        info += " ";
        info += ai.currentAttackTarget != null && ai.currentAttackTarget.targetTransform != null ? "Attack Target: " + ai.currentAttackTarget.targetTransform.name + " Position: " + ai.currentAttackTarget.targetTransform.position : "Attack Target: null";
        info += "\n";
        info += "Target Visible: " + ai.DTargetVisible + " Target Reachable: " + ai.DTargetReachable + " Last Obstacle: " + ai.DLastObstacleName + "\n\n";
        info += "Main AI Loop Running: " + ai.DMainLoopRunning + "\n";
        info += "Agent Speed: " + ai.agent.speed + " Agent Running: " + !ai.agent.isStopped + " Agent Path Status: " + ai.agent.pathStatus + "\n";
        info += "Last path valid: " + ai.DLastPathState; 
        Handles.Label(ai.transform.position + Vector3.up * 2, info, style);
    }
}
