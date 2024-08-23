// IMPORTANT NOTE: THIS FILE HAS BEEN DEPRECATED AND CONTENT HAS BEEN REINTEGRATED INTO GhostAI.cs
// FOR PRIOR IMPLEMENTATIONS SEE CL 635 BEFORE ADDITION OF STATE MACHINE IMPLEMENTATIONS
// THIS WAS DONE BECAUSE CERTAIN AI BEHAVIORS COULD TICK AT LESS FREQUENT RATES, NEED THE ABILITY TO JUMP TO A NON-REGULAR STATE 
// GIVEN A CONDITION TRIGGERED BY OUTSIDE FACTORS AND TURN OFF SOME OF THE EXECUTION FUNCTIONS FOR PERFORMANCE

//using System.Collections;
//using System.Collections.Generic;
//using UnityEngine;
//using Mirror;
//public class BaseAI : NetworkBehaviour
//{
//    [Header("Reference Settings")]
//    public Transform head;
//    [Header("Vision Settings")]
//    public LayerMask targetMask; //For players
//    public LayerMask obstacleMask; //For walls/blocks etc
//    public float viewRadius = 10f;
//    [Range(0, 360)]
//    public float viewAngle = 145f;

//    private NodeExecuter executer;

//    #region Getters
//    [Server]
//    public NodeExecuter GetExecuter()
//    {
//        return executer;
//    }

//    public override void OnStartServer()
//    {
//        base.OnStartServer();
//        executer = new NodeExecuter();
//        executer.StartTick(0.3f);
//    }
//    #endregion

//}

//public class NodeExecuter : BaseAI
//{
//    //public Stack<BaseAINode> repeatStack;
//    //public BaseAINode root;

//    //public BaseAINode tempCache;
//    //public BaseAINode current; //Pointer to current node pending execution

//    public bool isExecuting = false;
//    #region Node Callbacks / Events
//    //Called when all nodes connected have been evaluated and there isn't any next node
//    public void OnExecuteFinish()
//    {
//        if(repeatStack.Count == 0)
//        {
//            isExecuting = false;
//        }
//        else
//        {
//            isExecuting = true;
//            ExecuteTree(); //Executes over all repeats then stops and wait for next global tree tick
//        }

//        Debug.Log("Node completed tree execution! Overall Finished: " + isExecuting);
//    }

//    public void OnRemoveStackRepeatingNode()
//    {
//        if(repeatStack.Peek() != null)
//        {
//            repeatStack.Pop();
//        }
//    }
//    //Callback from executed note
//    public void OnFinishedCurrentNodeExecution()
//    {
//        if (current.IsEndNode())
//        {
//            OnExecuteFinish();
//        }
//        else
//        {
//            ExecuteNode(current.GetNextNode());
//        }
//    }

//    #endregion
//    public void ExecuteNode(BaseAINode node)
//    {
//        current = node;
//        current.OnNodeEnter();
//    }
//    public void ExecuteTree()
//    {
//        Debug.Log("Starting Tree Execution!");
//        isExecuting = true;
//        //Handle repeats
//        if (repeatStack.Count > 0)
//        {
//            ExecuteNode(repeatStack.Peek());
//            return;
//        }

//        ExecuteNode(root);

//    }

//    #region Main Controls
//    public void StartTick(float tickRate)
//    {
//        InvokeRepeating("ExecuteTree", 0, tickRate);
//    }

//    public void StopTick()
//    {
//        CancelInvoke();
//    }

//    #endregion
//}
