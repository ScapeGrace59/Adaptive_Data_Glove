using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;
using System.IO;
using System;

public class handControl : MonoBehaviour
{
    GameObject thumb1;
    GameObject thumb2;
    GameObject thumb3;
    GameObject index1;
    GameObject index2;
    GameObject index3;
    GameObject middle1;
    GameObject middle2;
    GameObject middle3;
    GameObject ring1;
    GameObject ring2;
    GameObject ring3;
    GameObject pinky1;
    GameObject pinky2;
    GameObject pinky3;
    GameObject wrist;
    GameObject hand;

    serial serialController;


    float maxjoint1;
    float maxjoint2;
    float maxjoint3;
    float minjoint1;
    float minjoint2;
    float minjoint3;
    private Quaternion currentRotation;

    float[] left_hand_rot = new float[] { 0.0f, 0.0f, 0.0f };


    int x, y, z;

    private List<float> timeStamps = new List<float>();
    private List<float> angleXValues = new List<float>();
    private List<float> angleYValues = new List<float>();
    private List<float> angleZValues = new List<float>();
    private List<float> objectangleXValues = new List<float>();
    private List<float> objectangleYValues = new List<float>();
    private List<float> objectangleZValues = new List<float>();

    // Start is called before the first frame update
    void Start()
    {	
	
		//Grab game objects for each "bone" in the hand skeleton
        thumb1 = GameObject.Find("b_l_thumb1").gameObject;
        thumb2 = GameObject.Find("b_l_thumb2").gameObject;
        thumb3 = GameObject.Find("b_l_thumb3").gameObject;

        index1 = GameObject.Find("b_l_index1").gameObject;
        index2 = GameObject.Find("b_l_index2").gameObject;
        index3 = GameObject.Find("b_l_index3").gameObject;

        middle1 = GameObject.Find("b_l_middle1").gameObject;
        middle2 = GameObject.Find("b_l_middle2").gameObject;
        middle3 = GameObject.Find("b_l_middle3").gameObject;

        ring1 = GameObject.Find("b_l_ring1").gameObject;
        ring2 = GameObject.Find("b_l_ring2").gameObject;
        ring3 = GameObject.Find("b_l_ring3").gameObject;

        pinky1 = GameObject.Find("b_l_pinky1").gameObject;
        pinky2 = GameObject.Find("b_l_pinky2").gameObject;
        pinky3 = GameObject.Find("b_l_pinky3").gameObject;

        wrist = GameObject.Find("OculusHand_L").gameObject;
        hand = GameObject.Find("OVRCustomHandPrefab_L").gameObject;
		
        serialController = GameObject.Find("commThread").GetComponent<serial>();

        minjoint1 = -5.0f;  //MCP Joint min angle
        minjoint2 = 0.0f;   //PIP Joint min angle
        minjoint3 = 8.0f;   //DIP Joint min angle
        maxjoint1 = -90.0f; //MCP Joint max angle
        maxjoint2 = -110.0f;//PIP Joint max angle
        maxjoint3 = -60.0f; //DIP joint max angle
        

    }

    // Update is called once per frame
    void Update()
    {

        for(int i = 0; i < 5; i++){
            float finger = serialController.getFinger(i);
            BendFinger(i, finger/100.0f);
        }

        rotate_hand();
        //translate_hand();

    }

	// Bend finger # to a certain percentage between fully curled and fully straight.
    void BendFinger(int finger, float percent){
        Transform tf1;
        Transform tf2;
        Transform tf3;

        float desired1 = percent * (maxjoint1 - minjoint1) + minjoint1;
        float desired2 = percent * (maxjoint2 - minjoint2) + minjoint2;
        float desired3 = percent * (maxjoint3 - minjoint3) + minjoint3;

        switch(finger){
            case 0:
                tf1 = thumb1.GetComponent<Transform>();
                tf2 = thumb2.GetComponent<Transform>();
                tf3 = thumb3.GetComponent<Transform>();
                break;
            case 1:
                tf1 = index1.GetComponent<Transform>();
                tf2 = index2.GetComponent<Transform>();
                tf3 = index3.GetComponent<Transform>();
                break;
            case 2:
                tf1 = middle1.GetComponent<Transform>();
                tf2 = middle2.GetComponent<Transform>();
                tf3 = middle3.GetComponent<Transform>();

                break;
            case 3:
                tf1 = ring1.GetComponent<Transform>();
                tf2 = ring2.GetComponent<Transform>();
                tf3 = ring3.GetComponent<Transform>();

                break;
            case 4:
                tf1 = pinky1.GetComponent<Transform>();
                tf2 = pinky2.GetComponent<Transform>();
                tf3 = pinky3.GetComponent<Transform>();

                break;
            default:
                return;

        }

        float angle1 = normalizeJointAngle(tf1.localEulerAngles.z);
        float angle2 = normalizeJointAngle(tf2.localEulerAngles.z);
        float angle3 = normalizeJointAngle(tf3.localEulerAngles.z);

        tf1.Rotate(0, 0, desired1-angle1);
        tf2.Rotate(0, 0, desired2-angle2);
        tf3.Rotate(0, 0, desired3-angle3);

        

        return;
    }

	//Unity passes angle without wrapping.  This functions allows for wrapping of angles
    float normalizeJointAngle(float angle){

        angle %= 360;
        if (angle > 180)
        {
            angle -= 360;
        }
        else if (angle < -180)
        {
            angle += 360;
        }
        return angle;
    }

    // Hand rotation about 3 axis
    void rotate_hand()
    {
        // Get the rotation values from the serialController
        float[] rotation = serialController.getWrist();

        float anglex = rotation[0] * 360;
        float angley = rotation[1] * 360;
        float anglez = rotation[2] * 360;
        
        hand.transform.Rotate(0, (anglez - left_hand_rot[2]), 0);
        left_hand_rot[2] = anglez;

        hand.transform.Rotate(0, 0, (anglex - left_hand_rot[0]));
        left_hand_rot[0] = anglex;

        hand.transform.Rotate(-(angley - left_hand_rot[1]), 0, 0);
        left_hand_rot[1] = angley;

        //Algorithm given below have to used if using Mahony Filter; above if using DMP

        /*float anglex = rotation[0];
        float angley = rotation[1];
        float anglez = rotation[2];

        anglex %= 40;
        anglex *= 9;
        angley %= 40;
        angley *= 9;
        anglez %= 40;
        anglez *= 9;

        hand.transform.Rotate(0, 0, (anglez - left_hand_rot[2]));
        left_hand_rot[2] = anglez;

        hand.transform.Rotate(0, (anglex - left_hand_rot[0]), 0);
        left_hand_rot[0] = anglex;

        hand.transform.Rotate(-(angley - left_hand_rot[1]), 0, 0);
        left_hand_rot[1] = angley;*/


        timeStamps.Add(Time.time);
        angleXValues.Add(anglex);
        angleYValues.Add(angley);
        angleZValues.Add(anglez);
        objectangleXValues.Add(hand.transform.rotation.eulerAngles.z);
        objectangleYValues.Add(hand.transform.rotation.eulerAngles.x);
        objectangleZValues.Add(hand.transform.rotation.eulerAngles.y);

    }

    // Hand translation about 3 axis
    void translate_hand()
    {
        float[] velocity = serialController.getHand();
        Vector3 currentVel = new Vector3(velocity[1], velocity[2], -velocity[0]);
        
        //hand.transform.position += currentVel * Time.deltaTime;
        hand.transform.Translate(currentVel * Time.deltaTime);

    }

    // Function to save the collected data to a file (You can arrange codes to plot finger angle values)
    public void SaveData()
    {
        string filePath = Application.dataPath + "/RotationData.csv";
        using (StreamWriter writer = new StreamWriter(filePath))
        {
            writer.WriteLine("Time,AngleX,AngleY,AngleZ,ObjectOrientationX,ObjectOrientationY,ObjectOrientationZ");

            for (int i = 0; i < timeStamps.Count; i++)
            {
                writer.WriteLine($"{timeStamps[i]};{angleXValues[i]};{angleYValues[i]};{angleZValues[i]};{objectangleXValues[i]};{objectangleYValues[i]};{objectangleZValues[i]}");
            }
        }

        Debug.Log("Rotation data saved to " + filePath);
    }
}
