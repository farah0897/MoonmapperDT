// ArenaBoundary.cs
//
// Definerer et rektangulaert test-omraade paa 20 m^2 (default 5 m x 4 m)
// oppaa Lunar Landscape-terrenget, med:
//   - Gizmo i Scene-view for visualisering
//   - Fire usynlige BoxCollider-vegger slik at roveren ikke trille ut
//   - Valgfritt synlige stolper i hjoernene som landmarks
//
// Plassering i Unity:
//   1. Opprett et tomt GameObject "ArenaBoundary" i scenen.
//   2. Plasser det paa et flatt sted paa maaneterrenget (juster Y til
//      ca. 0 - terrenghoeyden der).
//   3. Legg til dette scriptet.
//   4. Juster Size (X=lengde, Y=vegghoeyde, Z=bredde).
//   5. Huk paa Build Walls On Start for automatisk generering.

using UnityEngine;

[ExecuteAlways]
public class ArenaBoundary : MonoBehaviour
{
    [Header("Dimensjoner (meter)")]
    [Tooltip("Bredde (X), vegghoeyde (Y), dybde (Z). 5x4 = 20 m^2.")]
    public Vector3 size = new Vector3(5f, 0.3f, 4f);

    [Tooltip("Tykkelse paa veggkollidere (m).")]
    public float wallThickness = 0.05f;

    [Header("Vegger")]
    [Tooltip("Bygg fire usynlige BoxCollider-vegger rundt arenaet " +
             "ved scene-oppstart. Roveren kan dermed ikke falle ut.")]
    public bool buildWallsOnStart = true;

    [Tooltip(
        "Hvis true: veggene faar MeshRenderer saa de er synlige. " +
        "Useful hvis du vil se grensen i Game-view, ikke bare Scene.")]
    public bool wallsVisible = false;

    [Tooltip("Material paa synlige vegger (valgfri). Brukes kun hvis " +
             "wallsVisible = true.")]
    public Material wallMaterial;

    [Header("Hjornestolper")]
    [Tooltip("Plasser synlige stolper i hjoernene som SLAM-landmarks.")]
    public bool spawnCornerPosts = false;

    [Tooltip("Hoeyde paa stolper (m).")]
    public float postHeight = 0.3f;

    [Tooltip("Radius paa stolper (m).")]
    public float postRadius = 0.02f;

    [Tooltip("Farge paa stolper.")]
    public Color postColor = Color.red;

    private Transform _wallRoot;

    private void Start()
    {
        if (Application.isPlaying && buildWallsOnStart)
        {
            BuildWalls();
            if (spawnCornerPosts) SpawnPosts();
        }
    }

    private void BuildWalls()
    {
        // Slett ev. gamle vegger fra forrige Play-run
        var old = transform.Find("_Walls");
        if (old != null) DestroyImmediate(old.gameObject);

        _wallRoot = new GameObject("_Walls").transform;
        _wallRoot.SetParent(transform, worldPositionStays: false);

        float hx = size.x * 0.5f;
        float hz = size.z * 0.5f;
        float h  = size.y;
        float t  = wallThickness;

        // Fire vegger: +X, -X, +Z, -Z
        CreateWall("Wall+X", new Vector3( hx + t * 0.5f, h * 0.5f, 0f),
                             new Vector3(t, h, size.z + 2f * t));
        CreateWall("Wall-X", new Vector3(-hx - t * 0.5f, h * 0.5f, 0f),
                             new Vector3(t, h, size.z + 2f * t));
        CreateWall("Wall+Z", new Vector3(0f, h * 0.5f,  hz + t * 0.5f),
                             new Vector3(size.x, h, t));
        CreateWall("Wall-Z", new Vector3(0f, h * 0.5f, -hz - t * 0.5f),
                             new Vector3(size.x, h, t));
    }

    private void CreateWall(string name, Vector3 localPos, Vector3 lossyScale)
    {
        GameObject go;
        if (wallsVisible)
        {
            go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            if (wallMaterial != null)
                go.GetComponent<Renderer>().sharedMaterial = wallMaterial;
        }
        else
        {
            go = new GameObject(name);
            var bc = go.AddComponent<BoxCollider>();
            bc.size = Vector3.one;
        }
        go.name = name;
        go.transform.SetParent(_wallRoot, worldPositionStays: false);
        go.transform.localPosition = localPos;
        go.transform.localScale    = lossyScale;
    }

    private void SpawnPosts()
    {
        var postsRoot = new GameObject("_Posts").transform;
        postsRoot.SetParent(transform, worldPositionStays: false);

        float hx = size.x * 0.5f;
        float hz = size.z * 0.5f;

        Vector3[] corners =
        {
            new Vector3( hx, postHeight * 0.5f,  hz),
            new Vector3(-hx, postHeight * 0.5f,  hz),
            new Vector3( hx, postHeight * 0.5f, -hz),
            new Vector3(-hx, postHeight * 0.5f, -hz),
        };

        for (int i = 0; i < corners.Length; i++)
        {
            var post = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            post.name = $"Post_{i}";
            post.transform.SetParent(postsRoot, worldPositionStays: false);
            post.transform.localPosition = corners[i];
            post.transform.localScale = new Vector3(
                postRadius * 2f, postHeight * 0.5f, postRadius * 2f);

            var mat = new Material(Shader.Find("Standard"));
            mat.color = postColor;
            post.GetComponent<Renderer>().sharedMaterial = mat;
        }
    }

    // Scene-view gizmo slik at du ser arena-grensen uten aa kjoere Play.
    private void OnDrawGizmos()
    {
        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.color = new Color(1f, 0.7f, 0f, 1f); // oransje
        Gizmos.DrawWireCube(
            new Vector3(0f, size.y * 0.5f, 0f),
            new Vector3(size.x, size.y, size.z));

        // Fyllt halvtransparent bunn for tydelighet
        Gizmos.color = new Color(1f, 0.7f, 0f, 0.08f);
        Gizmos.DrawCube(
            new Vector3(0f, 0.005f, 0f),
            new Vector3(size.x, 0.01f, size.z));
    }
}
