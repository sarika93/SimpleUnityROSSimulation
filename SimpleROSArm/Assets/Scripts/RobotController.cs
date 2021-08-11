using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class RobotController : MonoBehaviour
{
  public ArticulationConfiguration ArticulationConfig;
  public float MaxJointVelDeg;
  public float[] JointGoalsDeg;
  public float[] CurrentAnglesDeg;
  public float[] CurrJointVelDeg;    //actual velocity
  public bool GoalReached;
  private UrdfJointRevolute[] _revoluteJoints;
  private ArticulationBody[] _revoluteArticulations;
  private int _numJoints;
  private float[] _jointVelsTargetDeg;  // what we want velocity to be


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
    JointGoalsDeg = new float[_numJoints];
  }

  [System.Serializable]
  public class ArticulationConfiguration
  {
    public float Stiffness;
    public float Damping;
    public float ForceLimit;
    public float Torque;
  }

  private void Start()
  {
    for (int i = 0; i < _numJoints; i++)
    {
      ArticulationDrive joint = _revoluteArticulations[i].xDrive;
      joint.damping = ArticulationConfig.Damping;
      joint.stiffness = ArticulationConfig.Stiffness;
      joint.forceLimit = ArticulationConfig.ForceLimit;

      _revoluteArticulations[i].xDrive = joint;
    }
    _jointVelsTargetDeg = new float[_numJoints];
    CurrJointVelDeg = new float[_numJoints];

    JointGoalsDeg = new float[6] { 90f, -90f, -90f, 63f, -34f, 156f };
  }

  private void FixedUpdate()
  {
    UpdateCurrentAnglesDeg();
    UpdateCurrentJointsVel();
    // Updates the joint velocities if Goal not reached
    UpdateJointsVel();

    if (!GoalReached)
    {
      for (int i = 0; i < _numJoints; i++)
      {
        ArticulationDrive drive = _revoluteArticulations[i].xDrive;
        float nextTarget = drive.target + Time.fixedDeltaTime * _jointVelsTargetDeg[i];
        if ((_jointVelsTargetDeg[i] > 0) && (Mathf.Abs(nextTarget) > Mathf.Abs(JointGoalsDeg[i])))
        {
          nextTarget = JointGoalsDeg[i];
        }
        else if ((_jointVelsTargetDeg[i] < 0) && (Mathf.Abs(nextTarget) < Mathf.Abs(JointGoalsDeg[i])))
        {
          nextTarget = JointGoalsDeg[i];
        }

        drive.target = nextTarget;
        _revoluteArticulations[i].xDrive = drive;
      }
    }
  }

  private void UpdateCurrentAnglesDeg()
  {
    for (int i = 0; i < _numJoints; i++)
    {
      CurrentAnglesDeg[i] = _revoluteJoints[i].GetPosition() * Mathf.Rad2Deg;
    }
  }

  private void UpdateCurrentJointsVel()
  {
    for (int i = 0; i < _numJoints; i++)
    {
      CurrJointVelDeg[i] = _revoluteJoints[i].GetVelocity() * Mathf.Rad2Deg;
    }
  }

  private void UpdateJointsVel()
  {
    float[] jointErrors = new float[_numJoints];
    float maxError = 0.0f;  // MaxError/MaxVel will give overall time.
    for (int i = 0; i < _numJoints; i++)
    {
      jointErrors[i] = JointGoalsDeg[i] - CurrentAnglesDeg[i];
      if (Mathf.Abs(jointErrors[i]) > maxError)
      {
        maxError = Mathf.Abs(jointErrors[i]);
      }
    }

    if (maxError < 0.001f)
    {
      GoalReached = true;
      for (int i = 0; i < _numJoints; i++)
      {
        _jointVelsTargetDeg[i] = 0f;
        // Goal reach so target should be current position
        ArticulationDrive drive = _revoluteArticulations[i].xDrive;
        drive.target = CurrentAnglesDeg[i];
        _revoluteArticulations[i].xDrive = drive;
      }
      return;
    }
    else
    {
      GoalReached = false;
    }

    // Some weirdness with this case. Check this out.
    if (MaxJointVelDeg == 0f)
    {
      for (int i = 0; i < _numJoints; i++)
      {
        _jointVelsTargetDeg[i] = 0f;
        ArticulationDrive drive = _revoluteArticulations[i].xDrive;
        drive.target = CurrentAnglesDeg[i];
        _revoluteArticulations[i].xDrive = drive;
      }
      return;
    }

    float time = maxError / MaxJointVelDeg;

    for (int i = 0; i < _numJoints; i++)
    {
      //Saving the velocities we want for each joint
      _jointVelsTargetDeg[i] = jointErrors[i] / time;
    }
  }
}
