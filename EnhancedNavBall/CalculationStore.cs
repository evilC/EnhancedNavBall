using UnityEngine;

public class CalculationStore
{
    public Vector3d RadialPlus;
    public Vector3d ProgradeOrbit;
    public Vector3d ProgradeSurface;
    public Vector3d OffsetProgradeSurface;
    public Vector3d OffsetRetrogradeSurface;
    public Vector3d NormalPlus;
    public Vector3d ManeuverPlus = Vector3d.zero;
    public bool ManeuverPresent;

    public void RunCalculations(
        Vessel vessel,
        Quaternion gymbal)
    {
        // Calculations thanks to Mechjeb
        Vector3d CoM = vessel.findWorldCenterOfMass();
        Vector3d up = (CoM - vessel.mainBody.position).normalized;
        Vector3d velocityVesselOrbit = vessel.orbit.GetVel();
        Vector3d velocityVesselOrbitUnit = velocityVesselOrbit.normalized;
        Vector3d radialPlus = Vector3d.Exclude(velocityVesselOrbit, up).normalized;
        Vector3d velocityVesselSurface = velocityVesselOrbit - vessel.mainBody.getRFrmVel(CoM);
        Vector3d velocityVesselSurfaceUnit = velocityVesselSurface.normalized;

        //Vector3d offsetProgradeSurface = Vector3d.Exclude(velocityVesselOrbit, up).normalized;
        //Vector3d offsetProgradeSurface = Vector3d.Exclude(velocityVesselOrbit, up).normalized;
        //Vector3d offsetProgradeSurface = Quaternion.AngleAxis(-45, Vector3.up);
        //Vector3d offsetProgradeSurface = velocityVesselSurfaceUnit;
        //Vector3d offsetProgradeSurface = Quaternion.Euler(0, -45, 0) * velocityVesselSurfaceUnit;
        Vector3d offsetProgradeSurface = Quaternion.AngleAxis(-90, Vector3.up) * velocityVesselSurfaceUnit;
        //Vector3d offsetRetrogradeSurface = velocityVesselSurfaceUnit;
        Vector3d offsetRetrogradeSurface = Quaternion.AngleAxis(90, Vector3.up) * velocityVesselSurfaceUnit;
        
        RadialPlus = gymbal * radialPlus;
        NormalPlus = gymbal * -Vector3d.Cross(radialPlus, velocityVesselOrbitUnit);
        ProgradeOrbit = gymbal * velocityVesselOrbitUnit;
        ProgradeSurface = gymbal * velocityVesselSurfaceUnit;
        OffsetProgradeSurface = gymbal * offsetProgradeSurface;
        OffsetRetrogradeSurface = gymbal * offsetRetrogradeSurface;

        if (vessel.patchedConicSolver.maneuverNodes.Count > 0)
        {
            Vector3d burnVector = vessel.patchedConicSolver.maneuverNodes[0].GetBurnVector(vessel.orbit);
            ManeuverPlus = gymbal * burnVector.normalized;
            ManeuverPresent = true;
        }
        else
        {
            ManeuverPresent = false;
        }
    }
}