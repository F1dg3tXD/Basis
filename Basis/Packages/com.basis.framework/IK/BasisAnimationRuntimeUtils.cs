using System;
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
    public static void SolveOne(
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
    public static void Apply(AnimationStream stream, ReadWriteTransformHandle h, Vector3Property p, Vector4Property r, Vector4Property o, BoolProperty sw)
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static Vector3 NormalizeSafe(Vector3 v)
    {
        float m2 = Vector3.Dot(v, v);
        if (m2 <= 1e-12f) return Vector3.forward;
        return v / Mathf.Sqrt(m2);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static Vector3 ProjectPointOnPlane(Vector3 point, Vector3 planePoint, Vector3 planeNormal)
    {
        // plane: (x - planePoint)·n = 0
        float d = Vector3.Dot(point - planePoint, planeNormal);
        return point - planeNormal * d;
    }

    /// <summary>
    /// Spine IK that tolerates missing spine/upperChest links.
    /// Active chain becomes: hips -> [spine?] -> chest -> [upperChest?] -> neck -> head
    /// Root can be pinned. Head matches target position; twist is distributed down the chain.
    /// Optional bend hint (usually chest forward) biases the chain into a plane.
    /// </summary>
    public static void SolveSpineChain(
        AnimationStream stream,
        // Required root & end:
        ReadWriteTransformHandle hips,
        ReadWriteTransformHandle chest,
        ReadWriteTransformHandle neck,
        ReadWriteTransformHandle head,
        // Optional middles (may be invalid in the stream):
        ReadWriteTransformHandle spine,         // optional
        ReadWriteTransformHandle upperChest,    // optional
                                                // Target & params
        AffineTransform headTarget,
        bool allowRootSlide,
        int iterations,
        float twistWeight,          // 0..1
        bool hasChestBendHint,      // if true, use bendHintDir
        Vector3 bendHintDir,        // world-space (e.g., chest forward)
        float bendBias              // 0..1 how strongly to bias toward bend plane
    )
    {
        // Validate minimally required joints
        if (!(hips.IsValid(stream) && chest.IsValid(stream) && neck.IsValid(stream) && head.IsValid(stream)))
        {
            // Not enough to solve a spine; pass through gracefully
            Pass(stream, hips, chest, head);
            PassThrough(stream, neck);
            return;
        }

        // ---- Build compacted chain in order without allocations ----
        // Handles h0..h5 (max 6 joints). We always include hips, chest, neck, head; optionally spine, upperChest.
        ReadWriteTransformHandle h0 = hips;
        ReadWriteTransformHandle h1, h2, h3, h4, h5;
        int count = 0;

        // We’ll push joints sequentially
        ReadWriteTransformHandle a0 = hips;
        ReadWriteTransformHandle a1 = spine.IsValid(stream) ? spine : chest; // If spine missing, chest takes slot 1
        ReadWriteTransformHandle a2, a3, a4, a5;

        if (spine.IsValid(stream))
        {
            // chain: hips(0) -> spine(1) -> chest(?)
            a2 = chest;
        }
        else
        {
            // chain: hips(0) -> chest(1)
            a2 = neck; // we’ll overwrite below if upperChest is valid
        }

        // Decide remaining based on which of spine/upperChest exist
        if (spine.IsValid(stream))
        {
            if (upperChest.IsValid(stream))
            {
                // hips, spine, chest, upperChest, neck, head
                a3 = upperChest; a4 = neck; a5 = head;
            }
            else
            {
                // hips, spine, chest, neck, head
                a3 = neck; a4 = head; a5 = head; // a5 dummy; will be ignored
            }
        }
        else
        {
            if (upperChest.IsValid(stream))
            {
                // hips, chest, upperChest, neck, head
                a2 = chest; a3 = upperChest; a4 = neck; a5 = head;
            }
            else
            {
                // hips, chest, neck, head
                a2 = chest; a3 = neck; a4 = head; a5 = head; // a5 dummy
            }
        }

        // Now compact into h0..hN-1 uniquely and compute count
        h0 = a0; count = 1;

        void Push(ref ReadWriteTransformHandle slot, ReadWriteTransformHandle handle)
        {
            if (!handle.Equals(h0) && (count == 0 || !handle.Equals(slot)))
            {
                switch (count)
                {
                    case 1: h1 = handle; break;
                    case 2: h2 = handle; break;
                    case 3: h3 = handle; break;
                    case 4: h4 = handle; break;
                    case 5: h5 = handle; break;
                    default: return;
                }
                count++;
            }
        }

        // Initialize to avoid CS0165 (will overwrite through Push)
        h1 = h2 = h3 = h4 = h5 = hips;

        // Push in order, skipping duplicates that might have happened in “missing link” branches
        Push(ref h1, a1);
        Push(ref h2, a2);
        Push(ref h3, a3);
        if (a4.GetHashCode() != a3.GetHashCode()) Push(ref h4, a4);
        if (a5.GetHashCode() != a4.GetHashCode() && a5.GetHashCode() != a3.GetHashCode()) Push(ref h5, a5);

        // Clamp count [4..6]
        count = Mathf.Clamp(count, 4, 6);

        // Utility accessors by index
        Vector3 GetPos(int i)
        {
            switch (i)
            {
                case 0: return h0.GetPosition(stream);
                case 1: return h1.GetPosition(stream);
                case 2: return h2.GetPosition(stream);
                case 3: return (count > 3) ? h3.GetPosition(stream) : h2.GetPosition(stream);
                case 4: return (count > 4) ? h4.GetPosition(stream) : h3.GetPosition(stream);
                default: return (count > 5) ? h5.GetPosition(stream) : h4.GetPosition(stream);
            }
        }
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
        Quaternion GetRot(int i)
        {
            switch (i)
            {
                case 0: return h0.GetRotation(stream);
                case 1: return h1.GetRotation(stream);
                case 2: return h2.GetRotation(stream);
                case 3: return (count > 3) ? h3.GetRotation(stream) : h2.GetRotation(stream);
                case 4: return (count > 4) ? h4.GetRotation(stream) : h3.GetRotation(stream);
                default: return (count > 5) ? h5.GetRotation(stream) : h4.GetRotation(stream);
            }
        }
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

        // Read positions & originals
        Vector3 p0 = GetPos(0), p1 = GetPos(1), p2 = GetPos(2);
        Vector3 p3 = (count > 3) ? GetPos(3) : p2;
        Vector3 p4 = (count > 4) ? GetPos(4) : p3;
        Vector3 p5 = (count > 5) ? GetPos(5) : p4;

        Vector3 o0 = p0, o1 = p1, o2 = p2, o3 = p3, o4 = p4, o5 = p5;

        // Segment lengths (count-1 segments)
        float L0 = (p1 - p0).magnitude;
        float L1 = (p2 - p1).magnitude;
        float L2 = (count > 3) ? (p3 - p2).magnitude : 0f;
        float L3 = (count > 4) ? (p4 - p3).magnitude : 0f;
        float L4 = (count > 5) ? (p5 - p4).magnitude : 0f;

        // Validate lengths
        if (L0 <= 1e-7f || L1 <= 1e-7f || (count > 3 && L2 <= 1e-7f) || (count > 4 && L3 <= 1e-7f) || (count > 5 && L4 <= 1e-7f))
        {
            // Avoid NaNs
            for (int i = 0; i < count; i++) SetPos(i, GetPos(i));
            return;
        }

        Quaternion targetRot = headTarget.rotation;
        Vector3 targetPos = headTarget.translation;

        // ---- FABRIK iterations ----
        int iters = Mathf.Max(1, iterations);
        for (int it = 0; it < iters; it++)
        {
            // Backward: set end to target, pull back
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
                    p2 = p3 + NormalizeSafe(p2 - p3) * L2; // note: in 4-link, L2 is (p3-p2) length; we repurpose vars below
                    p1 = p2 + NormalizeSafe(p1 - p2) * L1;
                    p0 = p1 + NormalizeSafe(p0 - p1) * L0;
                    break;
            }

            // Bend plane bias (optional). We bias interior joints toward a plane defined by axis & bendHintDir.
            if (hasChestBendHint && bendBias > 0f)
            {
                float bias = Mathf.Clamp01(bendBias);
                // axis root->end after backward
                Vector3 end = (count == 6) ? p5 : (count == 5) ? p4 : p3;
                Vector3 axis = NormalizeSafe(end - p0);
                // Plane normal from axis and hint dir (if parallel, fallback to up)
                Vector3 n = Vector3.Cross(axis, bendHintDir);
                if (n.sqrMagnitude < 1e-8f) n = Vector3.Cross(axis, Vector3.up);
                n = NormalizeSafe(n);

                // Project interior joints (excluding root & end) toward the plane through root with normal n
                if (count >= 4)
                {
                    p1 = Vector3.Lerp(p1, ProjectPointOnPlane(p1, p0, n), bias * 0.5f);
                    p2 = Vector3.Lerp(p2, ProjectPointOnPlane(p2, p0, n), bias * 0.7f);
                    if (count > 3) p3 = Vector3.Lerp(p3, ProjectPointOnPlane(p3, p0, n), bias * 0.9f);
                    if (count > 4) p4 = Vector3.Lerp(p4, ProjectPointOnPlane(p4, p0, n), bias * 1.0f);
                }
            }

            // Forward: pin/move root, push forward
            if (!allowRootSlide) p0 = o0;

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
        }

        // Write positions back
        SetPos(0, p0); SetPos(1, p1); SetPos(2, p2);
        if (count > 3) SetPos(3, p3);
        if (count > 4) SetPos(4, p4);
        if (count > 5) SetPos(5, p5);

        // Orient each joint to face its child (minimal swing; twist handled next)
        void FaceChild(int i, Vector3 oldA, Vector3 oldB, Vector3 newA, Vector3 newB)
        {
            Vector3 vOld = oldB - oldA;
            Vector3 vNew = newB - newA;
            if (vOld.sqrMagnitude <= 1e-12f || vNew.sqrMagnitude <= 1e-12f) return;
            Quaternion delta = QuaternionExt.FromToRotation(vOld, vNew);
            SetRot(i, delta * GetRot(i));
        }

        if (count == 6)
        {
            FaceChild(0, o0, o1, p0, p1);
            FaceChild(1, o1, o2, p1, p2);
            FaceChild(2, o2, o3, p2, p3);
            FaceChild(3, o3, o4, p3, p4);
            FaceChild(4, o4, o5, p4, p5);
            // Head rotation: match target
            SetRot(5, headTarget.rotation);
        }
        else if (count == 5)
        {
            FaceChild(0, o0, o1, p0, p1);
            FaceChild(1, o1, o2, p1, p2);
            FaceChild(2, o2, o3, p2, p3);
            FaceChild(3, o3, o4, p3, p4);
            SetRot(4, headTarget.rotation);
        }
        else // 4
        {
            FaceChild(0, o0, o1, p0, p1);
            FaceChild(1, o1, o2, p1, p2);
            FaceChild(2, o2, o3, p2, p3);
            SetRot(3, headTarget.rotation);
        }

        // Distribute twist from head target down the chain
        float tWeight = Mathf.Clamp01(twistWeight);
        if (tWeight > 0f)
        {
            // Compare current head rot vs desired
            Quaternion headNow = (count == 6) ? GetRot(5) : (count == 5) ? GetRot(4) : GetRot(3);
            Quaternion twistDelta = headTarget.rotation * Quaternion.Inverse(headNow);

            // Spread most near the neck; taper toward hips.
            // We only smear a portion 'tWeight' of the delta.
            float wNeck = 0.45f * tWeight;
            float wUp = 0.30f * tWeight;
            float wChest = 0.18f * tWeight;
            float wSpine = 0.07f * tWeight;

            if (count == 6)
            {
                SetRot(4, Quaternion.Slerp(Quaternion.identity, twistDelta, wNeck) * GetRot(4));
                SetRot(3, Quaternion.Slerp(Quaternion.identity, twistDelta, wUp) * GetRot(3));
                SetRot(2, Quaternion.Slerp(Quaternion.identity, twistDelta, wChest) * GetRot(2));
                SetRot(1, Quaternion.Slerp(Quaternion.identity, twistDelta, wSpine) * GetRot(1));
            }
            else if (count == 5)
            {
                SetRot(3, Quaternion.Slerp(Quaternion.identity, twistDelta, wNeck + 0.08f) * GetRot(3));
                SetRot(2, Quaternion.Slerp(Quaternion.identity, twistDelta, wUp + 0.05f) * GetRot(2));
                SetRot(1, Quaternion.Slerp(Quaternion.identity, twistDelta, wChest) * GetRot(1));
            }
            else // 4
            {
                SetRot(2, Quaternion.Slerp(Quaternion.identity, twistDelta, wNeck + 0.15f) * GetRot(2));
                SetRot(1, Quaternion.Slerp(Quaternion.identity, twistDelta, wUp + 0.10f) * GetRot(1));
            }
        }
    }
}
