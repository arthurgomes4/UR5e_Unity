using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class ROSTopicBasedControlPlugin : MonoBehaviour {

    ROSConnection ros;

    public ArticulationBody[] Joints;
    
    public string jointStatesTopic = "/joint_states";
    public string jointCommandsTopic = "/joint_command";

    public float frequency = 20f;

    private float TimeElapsed;
    private JointStateMsg stateMsg;
    private JointStateMsg commandMsg;

    private bool recvdCommandJointOrder = false;
    private int[] commandJointOrder;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(gameObject.transform.root.name+jointStatesTopic);
        ros.Subscribe<JointStateMsg>(gameObject.transform.root.name+jointCommandsTopic, CommandCallback);

        stateMsg = new JointStateMsg();
        commandMsg = new JointStateMsg();

        stateMsg.name = new string[Joints.Length];

        for (uint i = 0; i < Joints.Length; i++)
        {
            stateMsg.name[i] = Joints[i].GetComponent<UrdfJoint>().jointName;
        }

        stateMsg.position = new double[Joints.Length];
        stateMsg.velocity = new double[Joints.Length];
        stateMsg.effort = new double[Joints.Length];

        stateMsg.header = new HeaderMsg();
#if ROS2
#else
        stateMsg.header.seq = 0;
#endif

    }

    private void Update()
    {
        TimeElapsed += Time.deltaTime;

        if (TimeElapsed > 1/frequency)
        {
            float time = Time.time;
#if ROS2
            stateMsg.header.stamp.sec = (int)Math.Truncate(time);
#else
            stateMsg.header.stamp.sec = (uint)Math.Truncate(time);
            stateMsg.header.seq++;
#endif
            stateMsg.header.stamp.nanosec = (uint)((time - stateMsg.header.stamp.sec) * 1e+9);

            for (uint i = 0; i < Joints.Length; i++)
            {
                // 0 as the index as all of them are single degree of freedom joints.
                stateMsg.position[i] = (double)Joints[i].jointPosition[0];
                stateMsg.velocity[i] = (double)Joints[i].jointVelocity[0];
                stateMsg.effort[i] = (double)Joints[i].jointForce[0];
            }

            ros.Publish(gameObject.transform.root.name+jointStatesTopic, stateMsg);
            TimeElapsed = 0;
        }
    }

    void CommandCallback(JointStateMsg msg)
    {
        if (msg.position.Length != Joints.Length)
        {
            Debug.LogWarning("received message contains inavlid number of joints");
        }
        else
        {
            if (!recvdCommandJointOrder)
            {
                commandJointOrder = new int[msg.name.Length];
                for (int i = 0; i < msg.name.Length; i++)
                {
                    for (int j = 0; j < Joints.Length; j++)
                    {
                        if (msg.name[i] == Joints[j].GetComponent<UrdfJoint>().jointName)
                        {
                            commandJointOrder[i] = j;
                            break;
                        }
                    }
                }
                recvdCommandJointOrder = true;
            }
            else
            {
                for (int i = 0; i < msg.position.Length; i++)
                {
                    Joints[commandJointOrder[i]].SetDriveTarget(ArticulationDriveAxis.X, (float)msg.position[i] * Mathf.Rad2Deg);
                }
            }
        }
    }

}