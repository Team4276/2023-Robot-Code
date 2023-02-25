using UnityEngine;
using System.Collections;
using System.IO;
using System.Net.Http;
using System.Threading.Tasks;

public class MoveObject : MonoBehaviour
{
    public float speed = 1.0f;
    public float x, y, z, rx, ry, rz;
    public Vector3 targetPosition;


    void Start()
    {
        Update();
    }

    async void Update()
    {
        print("1");
        var client = new HttpClient();
        var response = await client.GetAsync("https://10.42.76.11:5807/request");

            // read the response and make it a string
            var jsonString = await response.Content.ReadAsStringAsync();
            print(jsonString);

            // parse json
            Results results = JsonUtility.FromJson<Results>(jsonString);

            // set coords
            x = results.ResultData.Fiducial[0].t6r_fs[0];
            y = results.ResultData.Fiducial[0].t6r_fs[1];
            z = results.ResultData.Fiducial[0].t6r_fs[2];
            rx = results.ResultData.Fiducial[0].t6r_fs[3];
            ry = results.ResultData.Fiducial[0].t6r_fs[4];
            rz = results.ResultData.Fiducial[0].t6r_fs[5];

            // update the destination
            targetPosition = new Vector3(x, y, z);
            transform.position = Vector3.MoveTowards(transform.position, targetPosition, speed * Time.deltaTime);
            
            // create a new quaternion axis using the euler angles
            Quaternion rotation = Quaternion.Euler(rx, ry, rz);
            // apply the rotation to the object
            transform.rotation = rotation;

    }
}

//tells unity what too deserialize from my understanding
[System.Serializable]
public class Results
{
    public Result ResultData;
}

[System.Serializable]
public class Result
{
    public Fiducial[] Fiducial;
}

[System.Serializable]
public class Fiducial
{
    public float[] t6r_fs;
}
