using System;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class SuspensionPoint
{
    public Transform point;
    public Transform visualObject;
    public Transform trackBone;
    public bool isStatic;
    public float wheelRadius = 0.235f;

    [HideInInspector] public float lastLength;
    [HideInInspector] public bool grounded;
}

public class TrackSuspension : MonoBehaviour
{
    private const float FORWARD_FRICTION = 5f;
    private const float SIDE_FRICTION = 2f;

    [SerializeField] private Rigidbody rb;
    [SerializeField] private LayerMask groundMask;
    [SerializeField] private SuspensionPoint[] points;
    [SerializeField] private Transform[] returnRollers;
    [SerializeField] private float returnRollerRadius = 0.1f;
    [SerializeField] private float restLength = 0.6f;
    [SerializeField] private float spring = 20000f;
    [SerializeField] private float damper = 3000f;
    [SerializeField] private float maxSpringForce = 10000f;
    [SerializeField] private float maxDamperForce = 5000f;
    [SerializeField] private float trackWidth = 0.05f;

    public int GroundedPoints { get; private set; }
    public bool IsGrounded => GroundedPoints > 0;
    public float Acceleration { get; set; }
    public bool Brake { get; set; } = true;
    public float ExtraVisualSpeed { get; set; } = 0f;
    public IReadOnlyList<SuspensionPoint> Points => points;
    public IReadOnlyList<Transform> ReturnRollers { get => returnRollers; }
    public float ReturnRollerRadius { get => returnRollerRadius; }

    public float GetTrackSpeed()
    {
        float totalSpeed = 0f;
        int count = 0;
        foreach (var sp in points)
        {
            if (!sp.grounded)
            {
                continue;
            }
            Vector3 r = sp.point.position - rb.worldCenterOfMass;
            Vector3 pointVel = rb.linearVelocity + Vector3.Cross(rb.angularVelocity, r);
            totalSpeed += transform.InverseTransformDirection(pointVel).z;
            count++;
        }
        return count > 0 ? totalSpeed / count : 0f;
    }

    void Update()
    {
        ApplyVisuals();
    }

    void FixedUpdate()
    {
        ApplySuspension();
        ApplyTraction();
        ApplySideFriction();
        Acceleration = 0f;
    }

    private void ApplyVisuals()
    {

        foreach (var sp in points)
        {
            if (sp.isStatic)
            {
                continue;
            }

            if (sp.visualObject != null)
            {
                float targetY = -sp.lastLength + sp.wheelRadius + trackWidth * 1.5f;
                Vector3 localPos = sp.visualObject.localPosition;
                localPos.y = Mathf.Lerp(localPos.y, targetY, 0.5f);
                sp.visualObject.localPosition = localPos;
            }

            if (sp.trackBone != null)
            {
                float targetY = -sp.lastLength + trackWidth / 2f;
                Vector3 localPos = sp.trackBone.localPosition;
                localPos.y = Mathf.Lerp(localPos.y, targetY, 0.5f);
                sp.trackBone.localPosition = localPos;
            }
        }

        if (!Brake)
        {
            float trackSpeed = GetTrackSpeed();

            foreach (var sp in points)
            {
                if (sp.visualObject == null)
                {
                    continue;
                }

                float rotationSpeed = trackSpeed / (2 * Mathf.PI * sp.wheelRadius) * 360f * Time.deltaTime;
                sp.visualObject.Rotate(Vector3.right, rotationSpeed + ExtraVisualSpeed, Space.Self);
            }

            float rollerRotationSpeed = trackSpeed / (2 * Mathf.PI * returnRollerRadius) * 360f * Time.deltaTime;
            foreach (var roller in returnRollers)
            {
                roller.Rotate(Vector3.right, rollerRotationSpeed + ExtraVisualSpeed, Space.Self);
            }
        }
    }

    private void ApplySuspension()
    {
        GroundedPoints = 0;

        float raycastOffset = restLength;

        foreach (var sp in points)
        {
            //if (sp.isStatic)
            //{
            //    continue;
            //}

            Vector3 rayOrigin = sp.point.position + transform.up * raycastOffset;
            float rayLength = restLength + raycastOffset;

            if (Physics.Raycast(
                rayOrigin,
                -transform.up,
                out RaycastHit hit,
                rayLength,
                groundMask))
            {
                float hitDistance = hit.distance - raycastOffset;
                hitDistance = Mathf.Clamp(hitDistance, 0f, restLength);

                float compression = restLength - hitDistance;
                float velocity = (sp.lastLength - hitDistance) / Time.fixedDeltaTime;

                float springForce = Mathf.Clamp(compression * spring, -maxSpringForce, maxSpringForce);
                float damperForce = Mathf.Clamp(velocity * damper, -maxDamperForce, maxDamperForce);

                float totalForce = springForce + damperForce;

                rb.AddForceAtPosition(transform.up * totalForce, sp.point.position);

                sp.lastLength = hitDistance;
                sp.grounded = true;
                GroundedPoints++;
            }
            else
            {
                sp.lastLength = restLength;
                sp.grounded = false;
            }
        }
    }

    private void ApplyTraction()
    {
        if (GroundedPoints == 0)
        {
            return;
        }

        float accelToApply = Acceleration;

        if (Brake)
        {
            ApplyForwardFriction();
            return;
        }
        else
        {
            Brake = false;
        }

        float accelPerPoint = accelToApply / GroundedPoints;

        foreach (var sp in points)
        {
            if (!sp.grounded)
            {
                continue;
            }

            rb.AddForceAtPosition(transform.forward * accelPerPoint, sp.point.position, ForceMode.Acceleration);
        }
    }
    private void ApplyForwardFriction()
    {
        foreach (var sp in points)
        {
            if (!sp.grounded)
            {
                continue;
            }

            Vector3 r = sp.point.position - rb.worldCenterOfMass;
            Vector3 pointVel = rb.linearVelocity + Vector3.Cross(rb.angularVelocity, r);

            Vector3 forwardVel = Vector3.Project(pointVel, transform.forward);

            float slip = Mathf.Clamp(1 / Mathf.Abs(5 * forwardVel.magnitude), 0.001f, 1f);

            rb.AddForceAtPosition(-forwardVel * FORWARD_FRICTION * slip, sp.point.position, ForceMode.Acceleration);
        }

    }

    private void ApplySideFriction()
    {
        foreach (var sp in points)
        {
            if (!sp.grounded)
            {
                continue;
            }

            Vector3 r = sp.point.position - rb.worldCenterOfMass;
            Vector3 pointVel = rb.linearVelocity + Vector3.Cross(rb.angularVelocity, r);

            Vector3 lateralVel = Vector3.Project(pointVel, transform.right);

            rb.AddForceAtPosition(-lateralVel * SIDE_FRICTION, sp.point.position, ForceMode.Acceleration);
        }
    }


#if UNITY_EDITOR
    void OnDrawGizmos()
    {
        if (points == null)
            return;
        foreach (var sp in points)
        {
            if (sp.visualObject != null)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawWireSphere(sp.visualObject.position, sp.wheelRadius);
            }
        }

        foreach (var roller in returnRollers)
        {
            Gizmos.DrawWireSphere(roller.position, returnRollerRadius);
        }
    }
#endif
}
