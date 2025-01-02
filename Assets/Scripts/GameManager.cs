using UnityEngine;
using UnityEngine.UI;

public class GameManager : MonoBehaviour
{
    public string ClassePerso;
    public GameObject Player;
    public bool GameStart = false;
    [SerializeField] private Animator PlayerAnimator;
    public RuntimeAnimatorController newController;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        if (GameStart)
        {
            PlayerAnimator.runtimeAnimatorController = newController;
            GameStart = false;
        }
    }
}
