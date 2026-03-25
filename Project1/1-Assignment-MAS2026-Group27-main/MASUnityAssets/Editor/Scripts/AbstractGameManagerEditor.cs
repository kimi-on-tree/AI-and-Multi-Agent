using System.Reflection;
using System.Text;
using UnityEditor;
using UnityEngine;
using Scripts.Game;

[CustomEditor(typeof(AbstractGameManager), true)]
public class AbstractGameManagerEditor : UnityEditor.Editor
{
    SerializedProperty _statistics;

    bool _showStats = true;

    const float NameColWidth = 240f;
    const float ValueColWidth = 180f;
    const float RowH = 18f;

    void OnEnable()
    {
        _statistics = serializedObject.FindProperty("statistics");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        DrawPropertiesExcluding(serializedObject, "m_Script", "statistics");

        EditorGUILayout.Space(10);
        DrawStatisticsTable();

        serializedObject.ApplyModifiedProperties();
    }

    void DrawStatisticsTable()
    {
        using (new EditorGUILayout.VerticalScope(EditorStyles.helpBox))
        {
            _showStats = EditorGUILayout.Foldout(_showStats, "Statistics", true);
            if (!_showStats) return;


            using (new EditorGUILayout.HorizontalScope())
            {
                GUILayout.FlexibleSpace();

                if (GUILayout.Button("Copy as TSV row (Excel)", GUILayout.Width(150)))
                {
                    EditorGUIUtility.systemCopyBuffer = BuildTsv();
                    Notify("Copied statistics as TSV");
                }
            }

            EditorGUILayout.Space(8);

            DrawHeaderRow("Metric", "Value");

            var fieldInfos = typeof(Statistics).GetFields(BindingFlags.Instance | BindingFlags.Public);


            using (new EditorGUI.DisabledScope(false))
            {
                foreach (var fieldInfo in fieldInfos)
                {
                    if (fieldInfo.Name == "planningTime" ||
                        fieldInfo.Name == "controllerTimeMs" || 
                        fieldInfo.Name == "numberOfCollisions" || 
                        fieldInfo.Name == "distanceTravelledLongest" ||
                        fieldInfo.Name == "topSpeed") EditorGUILayout.Space(4);
                    DrawRow(NiceWithUnits(fieldInfo.Name), GetFloat(fieldInfo.Name));
                }
            }
        }
    }

    string NiceWithUnits(string fieldName)
    {
        string s = ObjectNames.NicifyVariableName(fieldName);

        s = s.Replace(" Ms", " (ms)")
            .Replace(" Sec", " (s)")
            .Replace(" Fps", " FPS")
            .Replace(" Ai", " AI");

        return s;
    }

    void DrawHeaderRow(string a, string b)
    {
        var r = EditorGUILayout.GetControlRect(false, RowH);
        var left = new Rect(r.x, r.y, NameColWidth, r.height);
        var right = new Rect(r.x + NameColWidth, r.y, ValueColWidth, r.height);

        EditorGUI.LabelField(left, a, EditorStyles.boldLabel);
        EditorGUI.LabelField(right, b, EditorStyles.boldLabel);

        // A thin line
        var line = new Rect(r.x, r.yMax + 2, r.width, 1);
        EditorGUI.DrawRect(line, new Color(0, 0, 0, 0.2f));
        EditorGUILayout.Space(4);
    }

    void DrawRow(string metric, string value)
    {
        var r = EditorGUILayout.GetControlRect(false, RowH);
        var left = new Rect(r.x, r.y, NameColWidth, r.height);
        var right = new Rect(r.x + NameColWidth, r.y, ValueColWidth, r.height);

        EditorGUI.LabelField(left, metric);
        EditorGUI.SelectableLabel(right, value); // selectable is nice for quick copying too
    }

    // ---------- Data access / formatting ----------
    string GetFloat(string relativeName)
    {
        var p = _statistics.FindPropertyRelative(relativeName);
        if (p == null) return "—";
        return FormatFloat(p.floatValue);
    }

    static string FormatFloat(float v)
    {
        // Tweak format to taste
        return v.ToString("0.###");
    }

    string BuildTsv()
    {
        var sb = new StringBuilder();

        var fieldInfos = typeof(Statistics).GetFields(BindingFlags.Instance | BindingFlags.Public);

        foreach (var fieldInfo in fieldInfos)
        {
            sb.Append(GetFloat(fieldInfo.Name));
            if (fieldInfo != fieldInfos[fieldInfos.Length - 1]) sb.Append("\t");
        }

        return sb.ToString();
    }

    void Notify(string msg)
    {
        var w = EditorWindow.focusedWindow;
        if (w != null) w.ShowNotification(new GUIContent(msg));
        else Debug.Log(msg);
    }
}