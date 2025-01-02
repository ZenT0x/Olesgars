using UnityEngine;
using UnityEngine.UI;

public class SelectPerso : MonoBehaviour
{
    [SerializeField] private GameObject Canva;
    private string Classe;
    public GameManager Manager;
    private Animator animator;


    public void OnClick()
    {
        animator = GetComponent<Animator>();

        if (this.CompareTag("Voleur"))
        {
            Classe = "Voleur";
            Manager.newController = animator.runtimeAnimatorController;
        }

        if (this.CompareTag("Tueur"))
        {
            Classe = "Tueur";
            Manager.newController = animator.runtimeAnimatorController;
        }

        if (this.CompareTag("Arnaqueur"))
        {
            Classe = "Arnaqueur";
            Manager.newController = animator.runtimeAnimatorController;
        }

        if (this.CompareTag("Chercheur"))
        {
            Classe = "Chercheur";
            Manager.newController = animator.runtimeAnimatorController;
        }

        Manager.ClassePerso = Classe;
        Manager.GameStart = true;
        Canva.SetActive(false);
    }
}
