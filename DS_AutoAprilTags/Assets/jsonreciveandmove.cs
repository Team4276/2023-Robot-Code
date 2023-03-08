using UnityEngine;
using System.Net.Http;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

public class jsonreciveandmove : MonoBehaviour
{
    private float responseTime; // time elapsed since last response
    private GUIStyle guiStyle; // style for displaying text in top left of screen
    private string apiUrl = ""; // user input for API URL

    static readonly HttpClient client = new HttpClient();

    bool hasEnteredUrl = false; // flag to indicate if user has entered a URL

    void Start()
    {
        guiStyle = new GUIStyle();
        guiStyle.fontSize = 20;
        guiStyle.normal.textColor = Color.black;
    }

    async void Update()
    {
        if (hasEnteredUrl)
        {
            try
            {
                // measure time elapsed since last response
                float startTime = Time.realtimeSinceStartup;
                using HttpResponseMessage response = await client.GetAsync(apiUrl);
                responseTime = (Time.realtimeSinceStartup - startTime) * 1000; // convert to milliseconds

                string responseBody = await response.Content.ReadAsStringAsync();
                Debug.Log(responseBody);
                JObject jsonObject = JObject.Parse(responseBody);

                JArray t6r_fs = (JArray)jsonObject["Results"]["Fiducial"][0]["t6r_fs"];

                float x = (float)t6r_fs[0];
                float z = (float)t6r_fs[1];
                float y = (float)t6r_fs[2];
                float rx = (float)t6r_fs[3];
                float ry = (float)t6r_fs[4];
                float rz = (float)t6r_fs[5];

                Vector3 pos = new Vector3(x, y + 0.2f, z - 2.2f);
                Vector3 Rpos = new Vector3(rx, ry - 70f, rz);

                transform.position = pos;
                gameObject.transform.eulerAngles = Rpos;
                


            }
            catch (HttpRequestException e)
            {
                Debug.Log("exception caught");
                Debug.Log("message:");
                Debug.Log(e.Message);
            }
        }
    }

    void OnGUI()
    {
        // display response time in top left of screen
        GUI.Label(new Rect(10, 10, 200, 20), "Response Time: " + responseTime.ToString("F2") + " ms", guiStyle);

        // create input field for API URL
        GUILayout.BeginArea(new Rect(10, Screen.height - 44, Screen.width - 45, 40));
        GUILayout.BeginHorizontal();
        GUILayout.Label("API URL:", guiStyle, GUILayout.Width(77));
        apiUrl = GUILayout.TextField(apiUrl, GUILayout.Width(Screen.width - 12));
        if (Event.current.type == EventType.KeyUp && Event.current.keyCode == KeyCode.Return)
        {
            // if enter key is pressed, format the URL and set the flag to indicate that the user has entered a URL
            apiUrl = FormatApiUrl(apiUrl);
            hasEnteredUrl = true;
        }
        GUILayout.EndHorizontal();
        GUILayout.EndArea();
    }

    string FormatApiUrl(string url)
    {
        if (!url.StartsWith("http://") && !url.StartsWith("https://"))
        {
            url = "http://" + url;
        }

        if (!url.EndsWith(":5807/results"))
        {
            url += ":5807/results";
        }

        return url;
    }
}
