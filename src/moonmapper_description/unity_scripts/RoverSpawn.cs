// RoverSpawn.cs
//
// Teleporterer roveren til en spawn-transform ved Start og naar brukeren
// trykker en reset-tast (default R). Nulstiller ogsaa lineaer- og
// vinkelhastighet paa rot-ArticulationBody slik at roveren ikke har
// akkumulert fart etter en krasj.
//
// Plassering i Unity:
//   1. Lag et tomt GameObject "RoverSpawn" paa oensket startposisjon
//      (gjerne INNENFOR ArenaBoundary, ca. 0.10 m over bakken).
//   2. Legg dette scriptet paa "moonmapper"-roten (eller hvor som helst).
//   3. Dra RoverSpawn-transformen inn i "Spawn Point".
//   4. Dra "moonmapper"-GameObject inn i "Rover Root" og "Root Body"
//      (ArticulationBody paa base_link).

using UnityEngine;

public class RoverSpawn : MonoBehaviour
{
    [Header("Referanser")]
    [Tooltip("Tom Transform som markerer oensket startposisjon/-rotasjon.")]
    public Transform spawnPoint;

    [Tooltip("GameObject som skal flyttes (typisk `moonmapper`-roten).")]
    public Transform roverRoot;

    [Tooltip("ArticulationBody paa base_link. Brukes for aa nullstille " +
             "lineaer- og vinkelhastighet ved respawn.")]
    public ArticulationBody rootBody;

    [Header("Oppfoersel")]
    [Tooltip("Respawn roveren naar scenen starter.")]
    public bool spawnOnStart = true;

    [Tooltip("Reset-tast. Trykk for aa respawne midt under kjoering.")]
    public KeyCode resetKey = KeyCode.R;

    [Tooltip("Hvor mye ekstra loeft (m) som legges paa spawn Y " +
             "(slik at roveren ikke klemmer ned i bakken).")]
    public float verticalOffset = 0.05f;

    private void Start()
    {
        if (spawnOnStart) Respawn();
    }

    private void Update()
    {
        if (Input.GetKeyDown(resetKey)) Respawn();
    }

    public void Respawn()
    {
        if (spawnPoint == null || roverRoot == null)
        {
            Debug.LogError("[RoverSpawn] spawnPoint eller roverRoot er null.");
            return;
        }

        Vector3 pos = spawnPoint.position + Vector3.up * verticalOffset;
        Quaternion rot = spawnPoint.rotation;

        // ArticulationBody maa resettes via TeleportRoot, ellers henger
        // rigiditetsloeseren paa forrige tilstand og roveren "teleporterer"
        // tilbake til der den var foer flyttet.
        if (rootBody != null)
        {
            rootBody.TeleportRoot(pos, rot);
            rootBody.velocity = Vector3.zero;
            rootBody.angularVelocity = Vector3.zero;
            // Null ut ogsaa alle ledd-hastigheter
            foreach (var ab in rootBody.GetComponentsInChildren<ArticulationBody>())
            {
                ab.velocity = Vector3.zero;
                ab.angularVelocity = Vector3.zero;
                ab.jointVelocity = new ArticulationReducedSpace(0f, 0f, 0f);
            }
        }
        else
        {
            roverRoot.SetPositionAndRotation(pos, rot);
        }

        Debug.Log($"[RoverSpawn] Respawnet til {pos} (rot {rot.eulerAngles}).");
    }
}
