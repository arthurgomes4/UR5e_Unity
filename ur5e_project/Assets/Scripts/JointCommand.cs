using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System.Collections;

public class JointCommand : MonoBehaviour
{
    public string TopicName;

    public ArticulationBody[] Joints;

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float64MultiArrayMsg>(gameObject.transform.root.name + TopicName, Callback);        
    }

    void Callback(Float64MultiArrayMsg msg)
    {
        if (msg.data.Length != Joints.Length){
            Debug.LogWarning("received message contains inavlid number of joints");
        } else {
            for (int i=0; i < Joints.Length; i++) {
                Joints[i].SetDriveTarget(ArticulationDriveAxis.X, (float)msg.data[i] * Mathf.Rad2Deg);
            }
        }
    }
}