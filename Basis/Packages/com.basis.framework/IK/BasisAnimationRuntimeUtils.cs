using System;
using System.IO;
using System.Runtime.CompilerServices;
using UnityEngine;
using UnityEngine.Animations;
using UnityEngine.Animations.Rigging;

public static class BasisAnimationRuntimeUtils
{
    const float k_SqrEpsilon = 1e-8f;
    public static void SolveTwoBoneIKArms(
        AnimationStream stream,
        ReadWriteTransformHandle root,
        ReadWriteTransformHandle mid,
        ReadWriteTransformHandle tip,
        AffineTransform target,
        AffineTransform hint,
        bool hintWeight,
        AffineTransform targetOffset
    )
    {
        Vector3 aPosition = root.GetPosition(stream);
        Vector3 bPosition = mid.GetPosition(stream);
        Vector3 cPosition = tip.GetPosition(stream);

        Vector3 targetPos = target.translation;
        Quaternion targetRot = target.rotation;

        Vector3 tPosition = targetPos + targetOffset.translation;
        Quaternion tRotation = targetRot * targetOffset.rotation;

        Vector3 ab = bPosition - aPosition;
        Vector3 bc = cPosition - bPosition;
        Vector3 ac = cPosition - aPosition;
        Vector3 at = tPosition - aPosition;

        float abLen = ab.magnitude;
        float bcLen = bc.magnitude;
        float acLen = ac.magnitude;
        float atLen = at.magnitude;

        float oldAbcAngle = TriangleAngle(acLen, abLen, bcLen);
        float newAbcAngle = TriangleAngle(atLen, abLen, bcLen);

        // Prefer current bend plane; fallbacks to hint / at if collinear.
        Vector3 axis = Vector3.Cross(ab, bc);
        if (axis.sqrMagnitude < k_SqrEpsilon)
        {
            axis = hintWeight ? Vector3.Cross(hint.translation - aPosition, bc) : Vector3.zero;
            if (axis.sqrMagnitude < k_SqrEpsilon) axis = Vector3.Cross(at, bc);
            if (axis.sqrMagnitude < k_SqrEpsilon) axis = Vector3.up;
        }
        axis = Vector3.Normalize(axis);

        float a = 0.5f * (oldAbcAngle - newAbcAngle);
        float sin = Mathf.Sin(a);
        float cos = Mathf.Cos(a);
        Quaternion deltaR = new Quaternion(axis.x * sin, axis.y * sin, axis.z * sin, cos);
        mid.SetRotation(stream, deltaR * mid.GetRotation(stream));

        cPosition = tip.GetPosition(stream);
        ac = cPosition - aPosition;
        root.SetRotation(stream, QuaternionExt.FromToRotation(ac, at) * root.GetRotation(stream));

        if (hintWeight)
        {
            float acSqrMag = ac.sqrMagnitude;
            if (acSqrMag > 0f)
            {
                bPosition = mid.GetPosition(stream);
                cPosition = tip.GetPosition(stream);
                ab = bPosition - aPosition;
                ac = cPosition - aPosition;

                Vector3 acNorm = ac / Mathf.Sqrt(acSqrMag);
                Vector3 ah = hint.translation - aPosition;
                Vector3 abProj = ab - acNorm * Vector3.Dot(ab, acNorm);
                Vector3 ahProj = ah - acNorm * Vector3.Dot(ah, acNorm);

                float maxReach = abLen + bcLen;
                if (abProj.sqrMagnitude > (maxReach * maxReach * 0.001f) && ahProj.sqrMagnitude > 0f)
                {
                    Quaternion hintR = QuaternionExt.FromToRotation(abProj, ahProj);
                    hintR = QuaternionExt.NormalizeSafe(hintR);
                    root.SetRotation(stream, hintR * root.GetRotation(stream));
                }
            }
        }
        tip.SetRotation(stream, tRotation);
    }
    public static Vector3 ClosestPointOnSegment(Vector3 p, Vector3 a, Vector3 b)
    {
        Vector3 ab = b - a;
        float abSqr = Vector3.Dot(ab, ab);
        if (abSqr <= k_SqrEpsilon) return a;
        float t = Mathf.Clamp01(Vector3.Dot(p - a, ab) / abSqr);
        return a + ab * t;
    }
    public static void SegmentSegmentClosestPoints(
        Vector3 p1, Vector3 q1, Vector3 p2, Vector3 q2,
        out float s, out float t,
        out Vector3 c1, out Vector3 c2)
    {
        Vector3 d1 = q1 - p1;
        Vector3 d2 = q2 - p2;
        Vector3 r = p1 - p2;
        float a = Vector3.Dot(d1, d1);
        float e = Vector3.Dot(d2, d2);
        float f = Vector3.Dot(d2, r);

        if (a <= k_SqrEpsilon && e <= k_SqrEpsilon)
        {
            s = t = 0.0f; c1 = p1; c2 = p2; return;
        }
        if (a <= k_SqrEpsilon)
        {
            s = 0.0f; t = Mathf.Clamp01(f / e);
        }
        else
        {
            float c = Vector3.Dot(d1, r);
            if (e <= k_SqrEpsilon)
            {
                t = 0.0f; s = Mathf.Clamp01(-c / a);
            }
            else
            {
                float b = Vector3.Dot(d1, d2);
                float denom = a * e - b * b;

                if (denom != 0.0f) s = Mathf.Clamp01((b * f - c * e) / denom);
                else s = 0.0f;

                t = (b * s + f) / e;
                if (t < 0.0f) { t = 0.0f; s = Mathf.Clamp01(-c / a); }
                else if (t > 1.0f) { t = 1.0f; s = Mathf.Clamp01((b - c) / a); }
            }
        }

        c1 = p1 + d1 * s;
        c2 = p2 + d2 * t;
    }
    public static Vector3 CapsuleCapsuleResolve(Vector3 p1, Vector3 q1, float r1, Vector3 p2, Vector3 q2, float r2)
    {
        SegmentSegmentClosestPoints(p1, q1, p2, q2, out _, out _, out var c1, out var c2);
        Vector3 n = c1 - c2;
        float dSqr = Vector3.Dot(n, n);
        float rSum = r1 + r2;

        if (dSqr >= rSum * rSum) return Vector3.zero;

        Vector3 normal;
        if (dSqr > k_SqrEpsilon) normal = n / Mathf.Sqrt(dSqr);
        else
        {
            Vector3 axis = (q2 - p2);
            normal = Vector3.Normalize(Vector3.Cross(axis, Vector3.up));
            if (normal.sqrMagnitude < 1e-6f) normal = Vector3.Normalize(Vector3.Cross(axis, Vector3.right));
            if (normal.sqrMagnitude < 1e-6f) normal = Vector3.up;
        }

        float d = Mathf.Sqrt(Mathf.Max(dSqr, 0f));
        float penetration = (rSum - d);
        return normal * penetration;
    }
    public static void SwingElbowAroundAC(AnimationStream stream, ReadWriteTransformHandle root, ReadWriteTransformHandle mid, ReadWriteTransformHandle tip, Vector3 desiredB)
    {
        Vector3 A = root.GetPosition(stream);
        Vector3 C = tip.GetPosition(stream);
        Vector3 B = mid.GetPosition(stream);

        Vector3 AC = C - A;
        float acSqr = Vector3.Dot(AC, AC);
        if (acSqr <= k_SqrEpsilon) return;

        Vector3 n = AC / Mathf.Sqrt(acSqr);
        Vector3 v1 = B - A; v1 -= n * Vector3.Dot(v1, n);
        Vector3 v2 = desiredB - A; v2 -= n * Vector3.Dot(v2, n);

        float v1Sqr = Vector3.Dot(v1, v1);
        float v2Sqr = Vector3.Dot(v2, v2);
        if (v1Sqr <= k_SqrEpsilon || v2Sqr <= k_SqrEpsilon) return;

        v1 /= Mathf.Sqrt(v1Sqr);
        v2 /= Mathf.Sqrt(v2Sqr);

        float dot = Mathf.Clamp(Vector3.Dot(v1, v2), -1f, 1f);
        float ang = Mathf.Acos(dot);
        Vector3 cross = Vector3.Cross(v1, v2);
        float dir = Mathf.Sign(Vector3.Dot(cross, n));
        Quaternion swing = Quaternion.AngleAxis(ang * dir * Mathf.Rad2Deg, n);

        root.SetRotation(stream, swing * root.GetRotation(stream));
    }
    public static float TriangleAngle(float aLen, float aLen1, float aLen2)
    {
        float c = Mathf.Clamp((aLen1 * aLen1 + aLen2 * aLen2 - aLen * aLen) / (aLen1 * aLen2) / 2.0f, -1.0f, 1.0f);
        return Mathf.Acos(c);
    }
    public static void PassThrough(AnimationStream stream, ReadWriteTransformHandle handle)
    {
        handle.GetLocalTRS(stream, out Vector3 position, out Quaternion rotation, out Vector3 scale);
        handle.SetLocalTRS(stream, position, rotation, scale);
    }
    public static Vector3 PushOutFromCapsule(Vector3 p, Vector3 a, Vector3 b, float radiusWithSkin)
    {
        Vector3 q = ClosestPointOnSegment(p, a, b);
        Vector3 qp = p - q;
        float dSqr = Vector3.Dot(qp, qp);
        if (dSqr >= radiusWithSkin * radiusWithSkin) return p;
        float d = Mathf.Sqrt(Mathf.Max(dSqr, k_SqrEpsilon));
        Vector3 n = (d > 0f) ? (qp / d) : Vector3.up;
        return q + n * radiusWithSkin;
    }
    /// <summary>
    /// Evaluates the Two-Bone IK algorithm.
    /// </summary>
    /// <param name="stream">The animation stream to work on.</param>
    /// <param name="root">The transform handle for the root transform.</param>
    /// <param name="mid">The transform handle for the mid transform.</param>
    /// <param name="tip">The transform handle for the tip transform.</param>
    /// <param name="target">The transform handle for the target transform.</param>
    /// <param name="hint">The transform handle for the hint transform.</param>
    /// <param name="HasHint">The weight for which hint transform has an effect on IK calculations. This is a value in between 0 and 1.</param>
    /// <param name="targetOffset">The offset applied to the target transform.</param>
    public static void SolveTwoBone(AnimationStream stream, ReadWriteTransformHandle root, ReadWriteTransformHandle mid, ReadWriteTransformHandle tip, AffineTransform target, AffineTransform hint, bool HasHint, AffineTransform targetOffset, Vector3 BendNormal)
    {
        Vector3 aPosition = root.GetPosition(stream);
        Vector3 bPosition = mid.GetPosition(stream);
        Vector3 cPosition = tip.GetPosition(stream);

        Vector3 targetPos = target.translation;
        Quaternion targetRot = target.rotation;

        Vector3 tPosition = targetPos + targetOffset.translation;
        Quaternion tRotation = targetRot * targetOffset.rotation;

        Vector3 ab = bPosition - aPosition;
        Vector3 bc = cPosition - bPosition;
        Vector3 ac = cPosition - aPosition;
        Vector3 at = tPosition - aPosition;

        float abLen = ab.magnitude;
        float bcLen = bc.magnitude;
        float acLen = ac.magnitude;
        float atLen = at.magnitude;

        float oldAbcAngle = TriangleAngle(acLen, abLen, bcLen);
        float newAbcAngle = TriangleAngle(atLen, abLen, bcLen);
        Vector3 axis;
        if (HasHint)
        {
            axis = Vector3.Cross(hint.translation - aPosition, bc);

            if (axis.sqrMagnitude < k_SqrEpsilon)
            {
                axis = Vector3.Cross(at, bc);
            }

            if (axis.sqrMagnitude < k_SqrEpsilon)
            {
                axis = BendNormal;
            }
        }
        else
        {
            axis = BendNormal;
        }

        axis = Vector3.Normalize(axis);

        float halfAngle = 0.5f * (oldAbcAngle - newAbcAngle);
        float sin = Mathf.Sin(halfAngle);
        float cos = Mathf.Cos(halfAngle);
        Quaternion deltaR = new Quaternion(axis.x * sin, axis.y * sin, axis.z * sin, cos);
        mid.SetRotation(stream, deltaR * mid.GetRotation(stream));

        cPosition = tip.GetPosition(stream);
        ac = cPosition - aPosition;
        root.SetRotation(stream, QuaternionExt.FromToRotation(ac, at) * root.GetRotation(stream));

        if (HasHint)
        {
            float acSqrMag = ac.sqrMagnitude;
            if (acSqrMag > 0f)
            {
                bPosition = mid.GetPosition(stream);
                cPosition = tip.GetPosition(stream);
                ab = bPosition - aPosition;
                ac = cPosition - aPosition;

                Vector3 acNorm = ac / Mathf.Sqrt(acSqrMag);
                Vector3 ah = hint.translation - aPosition;
                Vector3 abProj = ab - acNorm * Vector3.Dot(ab, acNorm);
                Vector3 ahProj = ah - acNorm * Vector3.Dot(ah, acNorm);

                float maxReach = abLen + bcLen;
                if (abProj.sqrMagnitude > (maxReach * maxReach * 0.001f) && ahProj.sqrMagnitude > 0f)
                {
                    Quaternion hintR = QuaternionExt.FromToRotation(abProj, ahProj);
                    hintR = QuaternionExt.NormalizeSafe(hintR);
                    root.SetRotation(stream, hintR * root.GetRotation(stream));
                }
            }
        }

        tip.SetRotation(stream, tRotation);
    }
    public static Quaternion V4ToQuat(Vector4 v) => new Quaternion(v.x, v.y, v.z, v.w);
    public static void SolveLeg(
    AnimationStream stream,
    BoolProperty enabledProp,
    ReadWriteTransformHandle root, ReadWriteTransformHandle mid, ReadWriteTransformHandle tip,
    Vector3Property targetPosProp, Vector4Property targetRotProp,
    Vector3Property hintPosProp, Vector4Property hintRotProp,
    BoolProperty hintWeightProp, AffineTransform targetOffset, Vector3Property bendNormalProp)
    {
        if (!enabledProp.Get(stream))
        {
            Pass(stream, root, mid, tip);
            return;
        }

        if (!(root.IsValid(stream) && mid.IsValid(stream) && tip.IsValid(stream)))
        {
            Pass(stream, root, mid, tip);
            return;
        }

        Quaternion tRot = V4ToQuat(targetRotProp.Get(stream));
        Quaternion hRot = V4ToQuat(hintRotProp.Get(stream));

        AffineTransform target = new AffineTransform(targetPosProp.Get(stream), tRot);
        AffineTransform hint = new AffineTransform(hintPosProp.Get(stream), hRot);
        Vector3 bendNormal = bendNormalProp.Get(stream);

        BasisAnimationRuntimeUtils.SolveTwoBone(
            stream, root, mid, tip,
            target, hint,
            hintWeightProp.Get(stream),
            targetOffset, bendNormal
        );
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ApplyOverridenData(AnimationStream stream, ReadWriteTransformHandle h, Vector3Property p, Vector4Property r, Vector4Property o, BoolProperty sw)
    {
        if (h.IsValid(stream))
        {
            if (sw.Get(stream))
            {

                Vector3 targetPos = p.Get(stream);
                Vector4 rv4 = r.Get(stream);
                Vector4 ov4 = o.Get(stream);

                Quaternion targetRot = new Quaternion(rv4.x, rv4.y, rv4.z, rv4.w);
                Quaternion offsetRot = new Quaternion(ov4.x, ov4.y, ov4.z, ov4.w);

                Quaternion finalRot = targetRot * offsetRot;

                h.SetPosition(stream, targetPos);
                h.SetRotation(stream, finalRot);
            }
            else
            {
                BasisAnimationRuntimeUtils.PassThrough(stream, h);
            }
        }
    }
    public static void SolveHand(
    AnimationStream stream,
    BoolProperty enabledProp,
    ReadWriteTransformHandle root, ReadWriteTransformHandle mid, ReadWriteTransformHandle tip,
    Vector3Property targetPosProp, Vector4Property targetRotProp,
    Vector3Property hintPosProp, Vector4Property hintRotProp,
    BoolProperty hintWeightProp, AffineTransform targetOffset,
    ReadWriteTransformHandle chestStart, ReadWriteTransformHandle chestEnd,
    FloatProperty chestRadius, FloatProperty collisionSkin, BoolProperty collisionsEnabled,
    Vector3Property handLocalStart, Vector3Property handLocalEnd, FloatProperty handRadius, FloatProperty handSkin, BoolProperty useHandCapsule,
    BoolProperty protectElbow)
    {
        if (!enabledProp.Get(stream))
        {
            Pass(stream, root, mid, tip);
            return;
        }
        if (!(root.IsValid(stream) && mid.IsValid(stream) && tip.IsValid(stream)))
        {
            Pass(stream, root, mid, tip);
            return;
        }

        // Read inputs
        Vector3 tgtPos = targetPosProp.Get(stream);
        Quaternion tgtRot = V4ToQuat(targetRotProp.Get(stream));
        Vector3 hintPos = hintPosProp.Get(stream);
        Quaternion hintRot = V4ToQuat(hintRotProp.Get(stream));

        bool doCollisions = collisionsEnabled.Get(stream) && chestStart.IsValid(stream) && chestEnd.IsValid(stream);

        if (doCollisions)
        {
            Vector3 a = chestStart.GetPosition(stream);
            Vector3 b = chestEnd.GetPosition(stream);
            float chestR = Mathf.Max(0f, chestRadius.Get(stream) + collisionSkin.Get(stream));

            if (useHandCapsule.Get(stream))
            {
                Vector3 hsLocal = handLocalStart.Get(stream);
                Vector3 heLocal = handLocalEnd.Get(stream);
                float hRad = Mathf.Max(0f, handRadius.Get(stream) + handSkin.Get(stream));

                Vector3 handA = tgtPos + (tgtRot * hsLocal);
                Vector3 handB = tgtPos + (tgtRot * heLocal);

                Vector3 correction = BasisAnimationRuntimeUtils.CapsuleCapsuleResolve(handA, handB, hRad, a, b, chestR);
                if (correction.sqrMagnitude > 0f)
                {
                    tgtPos += correction;
                    hintPos += correction * 0.25f; // steer elbow slightly
                }
            }
            else
            {
                tgtPos = BasisAnimationRuntimeUtils.PushOutFromCapsule(tgtPos, a, b, chestR);
                Vector3 nudgedHint = BasisAnimationRuntimeUtils.PushOutFromCapsule(hintPos, a, b, chestR * 0.9f);
                hintPos = Vector3.Lerp(hintPos, nudgedHint, 0.6f);
            }
        }

        var target = new AffineTransform(tgtPos, tgtRot);
        var hint = new AffineTransform(hintPos, hintRot);

        // First solve (arms variant to preserve wrist)
        BasisAnimationRuntimeUtils.SolveTwoBoneIKArms(stream, root, mid, tip, target, hint, hintWeightProp.Get(stream), targetOffset);

        // Optional elbow protection pass
        if (protectElbow.Get(stream) && doCollisions)
        {
            Vector3 a = chestStart.GetPosition(stream);
            Vector3 b = chestEnd.GetPosition(stream);
            float chestR = Mathf.Max(0f, chestRadius.Get(stream) + collisionSkin.Get(stream));

            Vector3 B = mid.GetPosition(stream);
            Vector3 pushedB = BasisAnimationRuntimeUtils.PushOutFromCapsule(B, a, b, chestR);
            if ((pushedB - B).sqrMagnitude > 1e-10f)
            {
                BasisAnimationRuntimeUtils.SwingElbowAroundAC(stream, root, mid, tip, pushedB);
                // Re-lock wrist to target after elbow swing
                BasisAnimationRuntimeUtils.SolveTwoBoneIKArms(stream, root, mid, tip, target, hint, hintWeightProp.Get(stream), targetOffset);
            }
        }
    }
    public static void Pass(AnimationStream stream, ReadWriteTransformHandle root, ReadWriteTransformHandle mid, ReadWriteTransformHandle tip)
    {
        if (root.IsValid(stream)) BasisAnimationRuntimeUtils.PassThrough(stream, root);
        if (mid.IsValid(stream)) BasisAnimationRuntimeUtils.PassThrough(stream, mid);
        if (tip.IsValid(stream)) BasisAnimationRuntimeUtils.PassThrough(stream, tip);
    }

    public static void ApplyToeRotation(
        AnimationStream stream,
        BoolProperty enabledProp,
        ReadWriteTransformHandle handle,
        Vector3Property targetPosProp,
        Vector4Property targetRotProp)
    {
        if (!handle.IsValid(stream))
            return;

        if (enabledProp.Get(stream))
        {
            var pos = targetPosProp.Get(stream);
            var rot = V4ToQuat(targetRotProp.Get(stream));
            handle.SetPosition(stream, pos);
            handle.SetRotation(stream, rot);
        }
        else
        {
            BasisAnimationRuntimeUtils.PassThrough(stream, handle);
        }
    }

    // ---------- Small math utils ----------
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static Vector3 NormalizeSafe(Vector3 v, Vector3 fallback)
    {
        float m2 = Vector3.Dot(v, v);
        if (m2 <= 1e-12f) return fallback;
        return v * InverseSqrt(m2);
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float InverseSqrt(float x)
    {
        return x > 0f ? 1f / Mathf.Sqrt(x) : 0f;
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static Vector3 NormalizeSafe(Vector3 v) => NormalizeSafe(v, Vector3.forward);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Saturate(float x) => Mathf.Clamp01(x);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Unlerp(float a, float b, float v) => (v - a) / (b - a);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static Vector3 ProjectPointOnPlane(Vector3 point, Vector3 planePoint, Vector3 planeNormal)
    {
        float d = Vector3.Dot(point - planePoint, planeNormal);
        return point - planeNormal * d;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static Quaternion FromToRotation(Vector3 from, Vector3 to)
    {
        if (from.sqrMagnitude <= 1e-20f || to.sqrMagnitude <= 1e-20f)
            return Quaternion.identity;
        return Quaternion.FromToRotation(from, to);
    }

    // ---------- Optional guard/assist features ----------
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void LimitHipHeadDistance(ref Vector3 hip, ref Vector3 head, float maxLength, bool headDominates)
    {
        if (maxLength <= 0f) return;
        Vector3 d = head - hip;
        float L = d.magnitude;
        if (L <= maxLength) return;

        Vector3 dir = (L > 1e-8f) ? (d / L) : Vector3.forward;
        if (headDominates)
            hip = head - dir * maxLength;
        else
            head = hip + dir * maxLength;
    }

    /// <summary>
    /// Adds a small "tension" pushing hips away from head when near compression, to reduce buckling.
    /// </summary>
    static void ApplyBucklingTension(ref Vector3 hip, ref Vector3 head, Quaternion hipRot, Quaternion headRot,
                                     float maxLength, float gain /*0..1*/)
    {
        if (gain <= 0f || maxLength <= 0f) return;

        float dist = Vector3.Distance(hip, head);
        if (dist <= 1e-6f) return;

        // How compressed are we?
        float awayFromMax = 1f - dist / maxLength;                 // 0 (stretched) .. 1 (fully compressed)
        if (awayFromMax <= 0f) return;

        Vector3 tensionDir = NormalizeSafe(hip - head, Vector3.forward); // push hips away from head
        // Compare with a spine guide: blend hip/head local +X (like the other solver)
        Vector3 hipGuide = NormalizeSafe(hipRot * Vector3.right);
        Vector3 headGuide = NormalizeSafe(headRot * Vector3.right);
        Vector3 guide = NormalizeSafe(Vector3.Slerp(hipGuide, headGuide, 0.5f));

        float sim = Vector3.Dot(guide, tensionDir);                 // -1..1
        // We only want near-parallel (positive) and only when very aligned
        float sim01 = Saturate(Unlerp(0.96f, 1f, Mathf.Clamp(sim, -1f, 1f)));

        float total = awayFromMax * sim01 * gain;
        if (total <= 0f) return;

        Vector3 delta = tensionDir * total * maxLength * 0.05f;     // 5% of max span scaled by total
        hip += delta;                                               // nudge hips; we'll relimit below
        LimitHipHeadDistance(ref hip, ref head, maxLength, headDominates: true);
    }

    // ---------- Main solver ----------
    public static void SolveSpineChainWithHips(
        AnimationStream stream,

        // Hips drive
        in BoolProperty enabledHips,
        in ReadWriteTransformHandle handleHips,
        in Vector3Property targetPositionHips,
        in Vector4Property targetRotationHips,
        in Vector4Property offsetRotationHips,

        // Chain handles (some optional)
        ReadWriteTransformHandle handleChest,
        ReadWriteTransformHandle handleNeck,
        ReadWriteTransformHandle handleHead,
        ReadWriteTransformHandle handleSpine,        // optional
        ReadWriteTransformHandle handleUpperChest,   // optional

        // Head target (as properties)
        in Vector3Property targetPositionHead,
        in Vector4Property targetRotationHead,

        // IK tuning
        int iterations = 16,
        float twistWeight = 0.25f,
        bool hasChestBendHint = false,
        Vector3 bendHintDir = default,
        float bendBias = 1f,

        // -------- New optional knobs (ported ideas) --------
        // If clamped, should we move hips (true) or head (false)?
        bool headDominatesLimiter = true,
        // Anti-buckling nudge strength (0..1)
        float bucklingGain = 0.5f,
        // Early stop when the head gets sufficiently close to target (world units)
        float fabrikErrorEpsilon = 1e-4f
    )
    {
        // ---- 1) Hips write --------------------------------------------------------------
        if (handleHips.IsValid(stream))
        {
            if (enabledHips.Get(stream))
            {
                Vector3 hipPos = targetPositionHips.Get(stream);
                Quaternion hipRot = V4ToQuat(targetRotationHips.Get(stream));
                Quaternion hipOff = V4ToQuat(offsetRotationHips.Get(stream));
                handleHips.SetPosition(stream, hipPos);
                handleHips.SetRotation(stream, hipRot * hipOff); // offset in target space
            }
            else
            {
                BasisAnimationRuntimeUtils.PassThrough(stream, handleHips);
            }
        }

        // ---- 2) Validate chain ----------------------------------------------------------
        if (!(handleHips.IsValid(stream) &&
              handleChest.IsValid(stream) &&
              handleNeck.IsValid(stream) &&
              handleHead.IsValid(stream)))
        {
            BasisAnimationRuntimeUtils.Pass(stream, handleHips, handleChest, handleHead);
            BasisAnimationRuntimeUtils.PassThrough(stream, handleNeck);
            return;
        }

        // Build head target from properties
        var headTarget = new AffineTransform(
            targetPositionHead.Get(stream),
            V4ToQuat(targetRotationHead.Get(stream))
        );
        Quaternion targetRot = headTarget.rotation;
        Vector3 targetPos = headTarget.translation;

        // ---- 3) Compact chain (4..6 links) ----------------------------------------------
        ReadWriteTransformHandle h0 = default, h1 = default, h2 = default,
                                 h3 = default, h4 = default, h5 = default;
        int count = 0;
        void Push(ReadWriteTransformHandle h)
        {
            switch (count)
            {
                case 0: h0 = h; break;
                case 1: h1 = h; break;
                case 2: h2 = h; break;
                case 3: h3 = h; break;
                case 4: h4 = h; break;
                case 5: h5 = h; break;
            }
            count++;
        }
        Push(handleHips);
        if (handleSpine.IsValid(stream)) Push(handleSpine);
        Push(handleChest);
        if (handleUpperChest.IsValid(stream)) Push(handleUpperChest);
        Push(handleNeck);
        Push(handleHead);
        count = Mathf.Clamp(count, 4, 6);

        Vector3 GetPos(int i) => i switch
        {
            0 => h0.GetPosition(stream),
            1 => h1.GetPosition(stream),
            2 => h2.GetPosition(stream),
            3 => (count > 3) ? h3.GetPosition(stream) : h2.GetPosition(stream),
            4 => (count > 4) ? h4.GetPosition(stream) : h3.GetPosition(stream),
            _ => (count > 5) ? h5.GetPosition(stream) : h4.GetPosition(stream),
        };
        void SetPos(int i, Vector3 p)
        {
            switch (i)
            {
                case 0: h0.SetPosition(stream, p); break;
                case 1: h1.SetPosition(stream, p); break;
                case 2: h2.SetPosition(stream, p); break;
                case 3: if (count > 3) h3.SetPosition(stream, p); break;
                case 4: if (count > 4) h4.SetPosition(stream, p); break;
                case 5: if (count > 5) h5.SetPosition(stream, p); break;
            }
        }
        Quaternion GetRot(int i) => i switch
        {
            0 => h0.GetRotation(stream),
            1 => h1.GetRotation(stream),
            2 => h2.GetRotation(stream),
            3 => (count > 3) ? h3.GetRotation(stream) : h2.GetRotation(stream),
            4 => (count > 4) ? h4.GetRotation(stream) : h3.GetRotation(stream),
            _ => (count > 5) ? h5.GetRotation(stream) : h4.GetRotation(stream),
        };
        void SetRot(int i, Quaternion q)
        {
            switch (i)
            {
                case 0: h0.SetRotation(stream, q); break;
                case 1: h1.SetRotation(stream, q); break;
                case 2: h2.SetRotation(stream, q); break;
                case 3: if (count > 3) h3.SetRotation(stream, q); break;
                case 4: if (count > 4) h4.SetRotation(stream, q); break;
                case 5: if (count > 5) h5.SetRotation(stream, q); break;
            }
        }

        // ---- 4) Read positions/lengths --------------------------------------------------
        Vector3 p0 = GetPos(0), p1 = GetPos(1), p2 = GetPos(2);
        Vector3 p3 = (count > 3) ? GetPos(3) : p2;
        Vector3 p4 = (count > 4) ? GetPos(4) : p3;
        Vector3 p5 = (count > 5) ? GetPos(5) : p4;

        Vector3 o0 = p0, o1 = p1, o2 = p2, o3 = p3, o4 = p4, o5 = p5;

        float L0 = (p1 - p0).magnitude;
        float L1 = (p2 - p1).magnitude;
        float L2 = (count > 3) ? (p3 - p2).magnitude : 0f;
        float L3 = (count > 4) ? (p4 - p3).magnitude : 0f;
        float L4 = (count > 5) ? (p5 - p4).magnitude : 0f;

        if (L0 <= 1e-7f || L1 <= 1e-7f || (count > 3 && L2 <= 1e-7f) ||
            (count > 4 && L3 <= 1e-7f) || (count > 5 && L4 <= 1e-7f))
        {
            // preserve current pose; nothing to do
            for (int i = 0; i < count; i++) SetPos(i, GetPos(i));
            return;
        }

        // --- 4a) Hip↔Head limiter & anti-buckling (new) ---------------------------------
        // If no explicit max provided, you can derive a conservative one from the chain:
        float chainSum = L0 + L1 + L2 + L3 + L4;
        // We need current hip/hHead; “head” is the target, hips are p0.
        LimitHipHeadDistance(ref p0, ref targetPos, chainSum, headDominatesLimiter);

        // Anti-buckling nudge (uses current hip/head orientations)
        Quaternion hipNowRot = GetRot(0);
        ApplyBucklingTension(ref p0, ref targetPos, hipNowRot, targetRot, chainSum, bucklingGain);

        // Write back hip if we moved it (so stream drives subsequent reads consistently)
        SetPos(0, p0);

        // ---- 5) FABRIK -------------------------------------------------------------------
        int iters = Mathf.Max(1, iterations);
        float eps = Mathf.Max(1e-7f, fabrikErrorEpsilon);

        for (int it = 0; it < iters; it++)
        {
            // Backward: set end effector to (possibly limited) targetPos
            switch (count)
            {
                case 6:
                    p5 = targetPos;
                    p4 = p5 + NormalizeSafe(p4 - p5) * L4;
                    p3 = p4 + NormalizeSafe(p3 - p4) * L3;
                    p2 = p3 + NormalizeSafe(p2 - p3) * L2;
                    p1 = p2 + NormalizeSafe(p1 - p2) * L1;
                    p0 = p1 + NormalizeSafe(p0 - p1) * L0;
                    break;
                case 5:
                    p4 = targetPos;
                    p3 = p4 + NormalizeSafe(p3 - p4) * L3;
                    p2 = p3 + NormalizeSafe(p2 - p3) * L2;
                    p1 = p2 + NormalizeSafe(p1 - p2) * L1;
                    p0 = p1 + NormalizeSafe(p0 - p1) * L0;
                    break;
                default: // 4
                    p3 = targetPos;
                    p2 = p3 + NormalizeSafe(p2 - p3) * L2;
                    p1 = p2 + NormalizeSafe(p1 - p2) * L1;
                    p0 = p1 + NormalizeSafe(p0 - p1) * L0;
                    break;
            }

            // Bend plane hint (optional)
            if (hasChestBendHint && bendBias > 0f)
            {
                float bias = Mathf.Clamp01(bendBias);
                Vector3 end = (count == 6) ? p5 : (count == 5) ? p4 : p3;
                Vector3 axis = NormalizeSafe(end - p0, Vector3.forward);

                Vector3 n = Vector3.Cross(axis, bendHintDir);
                if (n.sqrMagnitude < 1e-8f) n = Vector3.Cross(axis, Vector3.up);
                n = NormalizeSafe(n, Vector3.up);

                if (count >= 4)
                {
                    p1 = Vector3.Lerp(p1, ProjectPointOnPlane(p1, p0, n), bias * 0.5f);
                    p2 = Vector3.Lerp(p2, ProjectPointOnPlane(p2, p0, n), bias * 0.7f);
                    if (count > 3) p3 = Vector3.Lerp(p3, ProjectPointOnPlane(p3, p0, n), bias * 0.9f);
                    if (count > 4) p4 = Vector3.Lerp(p4, ProjectPointOnPlane(p4, p0, n), bias * 1.0f);
                }
            }

            switch (count)
            {
                case 6:
                    p1 = p0 + NormalizeSafe(p1 - p0) * L0;
                    p2 = p1 + NormalizeSafe(p2 - p1) * L1;
                    p3 = p2 + NormalizeSafe(p3 - p2) * L2;
                    p4 = p3 + NormalizeSafe(p4 - p3) * L3;
                    p5 = p4 + NormalizeSafe(p5 - p4) * L4;
                    break;
                case 5:
                    p1 = p0 + NormalizeSafe(p1 - p0) * L0;
                    p2 = p1 + NormalizeSafe(p2 - p1) * L1;
                    p3 = p2 + NormalizeSafe(p3 - p2) * L2;
                    p4 = p3 + NormalizeSafe(p4 - p3) * L3;
                    break;
                default: // 4
                    p1 = p0 + NormalizeSafe(p1 - p0) * L0;
                    p2 = p1 + NormalizeSafe(p2 - p1) * L1;
                    p3 = p2 + NormalizeSafe(p3 - p2) * L2;
                    break;
            }

            // --- Early-out (new) ---
            Vector3 eff = (count == 6) ? p5 : (count == 5) ? p4 : p3;
            if ((eff - targetPos).sqrMagnitude <= eps * eps) break;
        }

        // Write positions back
        SetPos(0, p0); SetPos(1, p1); SetPos(2, p2);
        if (count > 3) SetPos(3, p3);
        if (count > 4) SetPos(4, p4);
        if (count > 5) SetPos(5, p5);

        // ---- 6) Minimal swing toward child; head = target rot ---------------------------
        void FaceChild(int i, Vector3 oldA, Vector3 oldB, Vector3 newA, Vector3 newB)
        {
            Vector3 vOld = oldB - oldA;
            Vector3 vNew = newB - newA;
            if (vOld.sqrMagnitude <= 1e-12f || vNew.sqrMagnitude <= 1e-12f) return;
            Quaternion delta = FromToRotation(vOld, vNew);
            SetRot(i, delta * GetRot(i));
        }

        if (count == 6)
        {
            FaceChild(0, o0, o1, p0, p1);
            FaceChild(1, o1, o2, p1, p2);
            FaceChild(2, o2, o3, p2, p3);
            FaceChild(3, o3, o4, p3, p4);
            FaceChild(4, o4, o5, p4, p5);
            SetRot(5, targetRot);
        }
        else if (count == 5)
        {
            FaceChild(0, o0, o1, p0, p1);
            FaceChild(1, o1, o2, p1, p2);
            FaceChild(2, o2, o3, p2, p3);
            FaceChild(3, o3, o4, p3, p4);
            SetRot(4, targetRot);
        }
        else // 4
        {
            FaceChild(0, o0, o1, p0, p1);
            FaceChild(1, o1, o2, p1, p2);
            FaceChild(2, o2, o3, p2, p3);
            SetRot(3, targetRot);
        }

        // ---- 7) Twist distribution (kept simple, safer weights) -------------------------
        float tWeight = Mathf.Clamp01(twistWeight);
        if (tWeight > 0f)
        {
            int headIdx = (count == 6) ? 5 : (count == 5) ? 4 : 3;
            Quaternion headNow = GetRot(headIdx);
            Quaternion twistDelta = targetRot * Quaternion.Inverse(headNow);

            // conservative falloff
            float wNeck = 0.45f * tWeight;
            float wUp = 0.30f * tWeight;
            float wChest = 0.18f * tWeight;
            float wSpine = 0.07f * tWeight;

            if (count == 6)
            {
                SetRot(4, Quaternion.SlerpUnclamped(Quaternion.identity, twistDelta, wNeck) * GetRot(4));
                SetRot(3, Quaternion.SlerpUnclamped(Quaternion.identity, twistDelta, wUp) * GetRot(3));
                SetRot(2, Quaternion.SlerpUnclamped(Quaternion.identity, twistDelta, wChest) * GetRot(2));
                SetRot(1, Quaternion.SlerpUnclamped(Quaternion.identity, twistDelta, wSpine) * GetRot(1));
            }
            else if (count == 5)
            {
                SetRot(3, Quaternion.SlerpUnclamped(Quaternion.identity, twistDelta, wNeck + 0.08f) * GetRot(3));
                SetRot(2, Quaternion.SlerpUnclamped(Quaternion.identity, twistDelta, wUp + 0.05f) * GetRot(2));
                SetRot(1, Quaternion.SlerpUnclamped(Quaternion.identity, twistDelta, wChest) * GetRot(1));
            }
            else // 4
            {
                SetRot(2, Quaternion.SlerpUnclamped(Quaternion.identity, twistDelta, wNeck + 0.15f) * GetRot(2));
                SetRot(1, Quaternion.SlerpUnclamped(Quaternion.identity, twistDelta, wUp + 0.10f) * GetRot(1));
            }
        }
    }
}
