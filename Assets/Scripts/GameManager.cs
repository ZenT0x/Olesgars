using UnityEngine;
using UnityEngine.UI;

public class GameManager : MonoBehaviour
{
    public string ClassePerso;
    public Color ColorSprite;
    public GameObject Player;
    private SpriteRenderer Sprite;
    public bool GameStart = false;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        Sprite = Player.GetComponent<SpriteRenderer>();
    }

    // Update is called once per frame
    void Update()
    {
        if (GameStart)
        {
            Sprite.color = ColorSprite;
            GameStart = false;
        }
    }
}
