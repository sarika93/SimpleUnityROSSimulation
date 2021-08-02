using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class RobotController : MonoBehaviour
{
  public float MaxJointVelDeg;
  public float[] CurrentAnglesDeg;
  public float[] JointGoalsDeg;  

  public UrdfJointRevolute[] _revoluteJoints;
  public ArticulationBody[] _revoluteArticulations;
  private int _numJoints;

  private void Awake()
  {
    // Finding all joints
    _revoluteJoints = GetComponentsInChildren<UrdfJointRevolute>();
    _numJoints = _revoluteJoints.Length;
    _revoluteArticulations = new ArticulationBody[_numJoints];
    CurrentAnglesDeg = new float[_numJoints];
    for (int i = 0; i < _numJoints; i++)
    {
      _revoluteArticulations[i] = _revoluteJoints[i].GetComponent<ArticulationBody>();
      if (_revoluteArticulations[i].jointType != ArticulationJointType.RevoluteJoint)
      {
        Debug.LogError("Not a revolute joint!");
      }
    }
  }

  private void Start()
  {

  }

  private void FixedUpdate()
  {
    UpdateCurrentAnglesDeg();
  }

  private void UpdateCurrentAnglesDeg()
  {
    for (int i=0; i < _numJoints; i++)
    {
      CurrentAnglesDeg[i] = _revoluteJoints[i].GetPosition() * Mathf.Rad2Deg;
    }
  }
}
