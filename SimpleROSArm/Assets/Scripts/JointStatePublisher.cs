using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStatePublisher : MonoBehaviour
{
  public string TopicName = "sensor_msgs/JointState";
  public float Frequency_Hz;
  private float _elapsedTimeSecs;
  private ROSConnection _ros;
  private RobotController _controller;

  // Joint names from the URDF
  private string[] _jointNames;

  private void Start()
  {
    if (Frequency_Hz <= 0.0f)
    {
      Debug.LogWarning("Frequency_Hz is set to invalid value 0Hz. Using default publish" +
        "rate of 25 Hz");
      Frequency_Hz = 25f;
    }
    _elapsedTimeSecs = 0f;
    // Finding robot controller to get robot state (should be on the same gameobject)
    _controller = GetComponent<RobotController>();

    // Start ROS connection
    _ros = ROSConnection.instance;
    _ros.RegisterPublisher<JointStateMsg>(TopicName);

     // TODO: Add in a method to grab these directly from the URDF
     _jointNames = new string[]{
       "shoulder_pan_joint",
       "shoulder_lift_joint",
       "elbow_joint",
       "wrist_1_joint",
       "wrist_2_joint",
       "wrist_3_joint",
     };
  }

  // Update is called once per frame
  private void FixedUpdate()
  {
    float period = 1f/Frequency_Hz;  // time between updates
    _elapsedTimeSecs += Time.fixedDeltaTime;  // update this to use actual time since
                                              // fixedDeltaTime is not actually fixed

    if (_elapsedTimeSecs >= period)
    {
      // http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html
      JointStateMsg jointState = new JointStateMsg();
      double epochTime = DateTime.Now.Subtract(new DateTime(1970, 1, 1)).TotalSeconds;
      jointState.header.stamp.sec = (uint)epochTime;
      jointState.header.stamp.nanosec = (uint)((epochTime%jointState.header.stamp.sec)* 1000000000);
      // Debug.Log("seconds: " + (jointState.header.stamp.sec).ToString() + " nsec: " +
      //   (jointState.header.stamp.nanosec).ToString());
      jointState.name = _jointNames;
      jointState.position = FloatToDoubleArray(_controller.CurrentAnglesRad);
      jointState.velocity = FloatToDoubleArray(_controller.CurrJointVelRad);

      _ros.Send(TopicName, jointState);
      _elapsedTimeSecs = 0f;
    }
  }

  // Articulation bodies use float by default, so need to be converted.
  private double[] FloatToDoubleArray(float[] array)
  {
    double[] doubleArray = new double[array.Length];
    for (int i = 0; i < array.Length; i++)
    {
      doubleArray[i] = (double)array[i];
    }
    return doubleArray;
  }
}
