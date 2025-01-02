using UnityEngine;
using UnityEngine.UI;

public class SelectPerso : MonoBehaviour
{
    [SerializeField] private GameObject Canva;
    private string Classe;
    [SerializeField] private Image image;
    public GameManager Manager;


    public void OnClick()
    {
        image = GetComponent<Image>();

        if (this.CompareTag("Voleur"))
        {
            Classe = "Voleur";
            Manager.ColorSprite = image.color;
        }

        if (this.CompareTag("Tueur"))
        {
            Classe = "Tueur";
            Manager.ColorSprite = image.color;
        }

        if (this.CompareTag("Arnaqueur"))
        {
            Classe = "Arnaqueur";
            Manager.ColorSprite = image.color;
        }

        if (this.CompareTag("Chercheur"))
        {
            Classe = "Chercheur";
            Manager.ColorSprite = image.color;
        }

        Manager.ClassePerso = Classe;
        Manager.GameStart = true;
        Canva.SetActive(false);
    }
}
