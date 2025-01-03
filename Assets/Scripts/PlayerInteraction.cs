using UnityEngine;

public class PlayerInteraction : MonoBehaviour
{
    public GameManager GameManager;
    private void OnTriggerEnter2D(Collider2D collision)
    {
        if (collision.CompareTag("Guard"))
        {
            GameManager.inCombat = true;
            GameManager.isStop = true;
            Debug.Log("Lancement du combat");
        }
    }
}
