using UnityEngine;
using UnityEngine.SceneManagement;

public class SwapScene : MonoBehaviour
{
    [SerializeField] private string SceneName;
    public void OnClick()
    {
        SceneManager.LoadScene(SceneName);
    }
}
