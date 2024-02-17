using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class JointState : MonoBehaviour {

    ROSConnection ros;

    public ArticulationBody[] Joints;
    
    public string TopicName = "/joint_states";
    public float PublishMessageFrequency = 5f;

    private float TimeElapsed;
    private JointStateMsg Message;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(gameObject.transform.root.name+TopicName);
        Message = new JointStateMsg();

        Message.name = new string[Joints.Length];

        for (uint i = 0; i < Joints.Length; i++)
        {
            Message.name[i] = Joints[i].GetComponent<UrdfJoint>().jointName;
        }

        Message.position = new double[Joints.Length];
        Message.velocity = new double[Joints.Length];
        Message.effort = new double[Joints.Length];

        Message.header = new HeaderMsg();
#if ROS2
#else
        Message.header.seq = 0;
#endif

    }

    private void Update()
    {
        TimeElapsed += Time.deltaTime;

        if (TimeElapsed > 1/PublishMessageFrequency)
        {
            float time = Time.time;
#if ROS2
            Message.header.stamp.sec = (int)Math.Truncate(time);
#else
            Message.header.stamp.sec = (uint)Math.Truncate(time);
            Message.header.seq++;
#endif
            Message.header.stamp.nanosec = (uint)((time - Message.header.stamp.sec) * 1e+9);

            for (uint i = 0; i < Joints.Length; i++)
            {
                // 0 as the index as all of them are single degree of freedom joints.
                Message.position[i] = (double)Joints[i].jointPosition[0];
                Message.velocity[i] = (double)Joints[i].jointVelocity[0];
                Message.effort[i] = (double)Joints[i].jointForce[0];
            }

            ros.Publish(gameObject.transform.root.name+TopicName, Message);
            TimeElapsed = 0;
        }
    }

}