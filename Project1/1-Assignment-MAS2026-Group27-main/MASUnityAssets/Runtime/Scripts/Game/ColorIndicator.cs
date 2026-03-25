using System;
using UnityEngine;

namespace Scripts.Game
{
    public class ColorIndicator : MonoBehaviour
    {
        private static Color[] teamColors = { Color.black, Color.white, Color.cyan, Color.green, Color.yellow, Color.magenta, Color.gray, Color.red };

        public Renderer customRenderer;
        public Color initialColor;
        private bool colorSet;

        public void SetByIndex(int colorIndex)
        {
            if (colorIndex >= 0 && colorIndex < teamColors.Length)
                SetColor(teamColors[colorIndex]);
        }

        public void SetColor(Color color)
        {
            initialColor = color;
            colorSet = true;
        }

        private void FixedUpdate()
        {
            if (!colorSet)
                return;

            if (customRenderer != null) customRenderer.material.color = initialColor;
            else gameObject.GetComponent<Renderer>().material.color = initialColor;
        }
    }
}