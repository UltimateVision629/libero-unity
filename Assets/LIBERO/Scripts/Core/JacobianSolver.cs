using UnityEngine;

namespace LIBERO.Core
{
    /// <summary>
    /// IK solver mode selector.
    /// DLS = original DampedLeastSquares (fast, single-step).
    /// LM_Chan / LM_Wampler = Levenberg-Marquardt iterative solver.
    /// </summary>
    public enum IKMode
    {
        DLS,
        LM_Chan,
        LM_Wampler
    }

    /// <summary>
    /// Result of an IK solve.
    /// </summary>
    public struct IKSolution
    {
        public float[] q;
        public bool success;
        public int iterations;
        public float residual;
        public string reason;

        public static IKSolution Fail(float[] q, float residual, string reason)
        {
            return new IKSolution { q = q, success = false, iterations = 0, residual = residual, reason = reason };
        }

        public static IKSolution Ok(float[] q, int iterations, float residual)
        {
            return new IKSolution { q = q, success = true, iterations = iterations, residual = residual, reason = "Success" };
        }
    }

    public static class JacobianSolver
    {
        // ════════════════════════════════════════════════════════════
        //  ORIGINAL DLS METHODS (preserved unchanged)
        // ════════════════════════════════════════════════════════════

        public static float[] DampedLeastSquares(ArticulationBody root, ArticulationBody[] joints,
            int dofCount, Vector3 deltaPos, Vector3 deltaRot, float lambda, float maxAngle)
        {
            int n = joints.Length;
            var jac = new ArticulationJacobian();
            root.GetDenseJacobian(ref jac);
            int rows = jac.rows;
            int cols = jac.columns;

            if (jac.elements == null || jac.elements.Count != rows * cols)
                return new float[n];

            var J = jac.elements;
            int eefStart = rows - dofCount;

            float[] A = new float[dofCount * dofCount];
            for (int i = 0; i < dofCount; i++)
            {
                for (int k = 0; k < dofCount; k++)
                {
                    float sum = 0f;
                    for (int j = 0; j < cols; j++)
                        sum += J[(eefStart + i) * cols + j] * J[(eefStart + k) * cols + j];
                    A[i * dofCount + k] = sum;
                }
            }

            float lambda2 = lambda * lambda;
            for (int i = 0; i < dofCount; i++)
                A[i * dofCount + i] += lambda2;

            float[] b = new float[dofCount];
            b[0] = deltaPos.x;
            b[1] = deltaPos.y;
            b[2] = deltaPos.z;
            b[3] = deltaRot.x;
            b[4] = deltaRot.y;
            b[5] = deltaRot.z;

            float[] x = GaussianSolve(A, b, dofCount);
            if (x == null) return new float[n];

            float[] dTheta = new float[n];
            int limit = System.Math.Min(n, cols);
            for (int j = 0; j < limit; j++)
            {
                float sum = 0f;
                for (int i = 0; i < dofCount; i++)
                    sum += J[(eefStart + i) * cols + j] * x[i];
                dTheta[j] = sum;
            }

            return dTheta;
        }

        /// <summary>
        /// Full 6-DOF damped least squares with locked joint.
        /// Solves IK for EEF pose (position + orientation) using joints excluding lockedJointIndex.
        /// dTheta[lockedJointIndex] is always 0.
        /// </summary>
        public static float[] DampedLeastSquaresWithLockedJoint(ArticulationBody root, ArticulationBody[] joints,
            int lockedJointIndex, Vector3 deltaPos, Vector3 deltaRot, float lambda, float maxAngle)
        {
            int n = joints.Length;
            var jac = new ArticulationJacobian();
            root.GetDenseJacobian(ref jac);

            if (jac.elements == null || jac.elements.Count != jac.rows * jac.columns)
                return new float[n];

            var J = jac.elements;
            int rows = jac.rows;
            int cols = jac.columns;
            int eefStart = rows - 6;  // last 6 rows = EEF twist

            int activeCols = cols - 1;
            int dofCount = 6;

            // Build reduced Jacobian (6 × activeCols)
            float[] Jred = new float[dofCount * activeCols];
            for (int i = 0; i < dofCount; i++)
            {
                int srcRow = eefStart + i;
                int dstCol = 0;
                for (int j = 0; j < cols; j++)
                {
                    if (j == lockedJointIndex) continue;
                    Jred[i * activeCols + dstCol] = J[srcRow * cols + j];
                    dstCol++;
                }
            }

            // A = Jred * Jred^T (6×6) + λ²I
            float[] A = new float[dofCount * dofCount];
            for (int i = 0; i < dofCount; i++)
            {
                for (int k = 0; k < dofCount; k++)
                {
                    float sum = 0f;
                    for (int j = 0; j < activeCols; j++)
                        sum += Jred[i * activeCols + j] * Jred[k * activeCols + j];
                    A[i * dofCount + k] = sum;
                }
            }

            float lambda2 = lambda * lambda;
            for (int i = 0; i < dofCount; i++)
                A[i * dofCount + i] += lambda2;

            float[] b = new float[] {
                deltaPos.x, deltaPos.y, deltaPos.z,
                deltaRot.x, deltaRot.y, deltaRot.z
            };

            float[] x = GaussianSolve(A, b, dofCount);
            if (x == null) return new float[n];

            // dTheta' = Jred^T * x
            float[] dThetaActive = new float[activeCols];
            for (int j = 0; j < activeCols; j++)
            {
                float sum = 0f;
                for (int i = 0; i < dofCount; i++)
                    sum += Jred[i * activeCols + j] * x[i];
                dThetaActive[j] = sum;
            }

            // Clamp
            for (int j = 0; j < activeCols; j++)
                dThetaActive[j] = Mathf.Clamp(dThetaActive[j], -maxAngle, maxAngle);

            // Map back to full joint array
            float[] dTheta = new float[n];
            int dst = 0;
            for (int j = 0; j < n; j++)
            {
                if (j == lockedJointIndex)
                    dTheta[j] = 0f;
                else
                    dTheta[j] = dThetaActive[dst++];
            }

            return dTheta;
        }

        /// <summary>
        /// Position-only damped least squares with locked joint.
        /// Solves IK for EEF position using only the joints excluding lockedJointIndex.
        /// dTheta[lockedJointIndex] is always 0.
        /// </summary>
        public static float[] DampedLeastSquaresPositionOnly(ArticulationBody root, ArticulationBody[] joints,
            int lockedJointIndex, Vector3 deltaPos, float lambda, float maxAngle)
        {
            int n = joints.Length;
            var jac = new ArticulationJacobian();
            root.GetDenseJacobian(ref jac);

            if (jac.elements == null || jac.elements.Count != jac.rows * jac.columns)
                return new float[n];

            var J = jac.elements;
            int rows = jac.rows;
            int cols = jac.columns;

            // Use first 3 rows (position) from the EEF part of the Jacobian
            int posStart = rows - 6;  // last 6 rows = EEF twist, first 3 = position

            int activeCols = cols - 1;
            float[] Jpos = new float[3 * activeCols];
            for (int i = 0; i < 3; i++)
            {
                int srcRow = posStart + i;
                int dstCol = 0;
                for (int j = 0; j < cols; j++)
                {
                    if (j == lockedJointIndex) continue;
                    Jpos[i * activeCols + dstCol] = J[srcRow * cols + j];
                    dstCol++;
                }
            }

            // A = Jpos * Jpos^T  (3×3)
            float[] A = new float[9];
            for (int i = 0; i < 3; i++)
            {
                for (int k = 0; k < 3; k++)
                {
                    float sum = 0f;
                    for (int j = 0; j < activeCols; j++)
                        sum += Jpos[i * activeCols + j] * Jpos[k * activeCols + j];
                    A[i * 3 + k] = sum;
                }
            }

            float lambda2 = lambda * lambda;
            for (int i = 0; i < 3; i++)
                A[i * 3 + i] += lambda2;

            float[] b = new float[] { deltaPos.x, deltaPos.y, deltaPos.z };
            float[] x = GaussianSolve(A, b, 3);
            if (x == null) return new float[n];

            // dTheta' = Jpos^T * x  (activeCols vector)
            float[] dThetaActive = new float[activeCols];
            for (int j = 0; j < activeCols; j++)
            {
                float sum = 0f;
                for (int i = 0; i < 3; i++)
                    sum += Jpos[i * activeCols + j] * x[i];
                dThetaActive[j] = sum;
            }

            // Clamp
            for (int j = 0; j < activeCols; j++)
                dThetaActive[j] = Mathf.Clamp(dThetaActive[j], -maxAngle, maxAngle);

            // Map back to full joint array
            float[] dTheta = new float[n];
            int dst = 0;
            for (int j = 0; j < n; j++)
            {
                if (j == lockedJointIndex)
                    dTheta[j] = 0f;
                else
                    dTheta[j] = dThetaActive[dst++];
            }

            return dTheta;
        }

        // ════════════════════════════════════════════════════════════
        //  LEVENBERG-MARQUARDT IK SOLVER
        //  Based on lerobot-kinematics IK_LM algorithm.
        // ════════════════════════════════════════════════════════════

        /// <summary>
        /// Solve IK using Levenberg-Marquardt with angle-axis error.
        ///
        /// Delegate signatures:
        ///   fkine(float[] q)      → Matrix4x4 (current EEF transform)
        ///   jacob0(float[] q)     → float[6][n] row-major Jacobian
        ///   getCurrentQ()         → float[] (current joint angles, rad)
        ///   applyDeltaQ(float[] dq) → void (apply delta to joints)
        /// </summary>
        public static IKSolution SolveLM(
            System.Func<float[], Matrix4x4> fkine,
            System.Func<float[], float[][]> jacob0,
            System.Func<float[]> getCurrentQ,
            System.Action<float[]> applyDeltaQ,
            int n,
            Matrix4x4 Tep,
            float ilimit = 10,
            float slimit = 5,
            float tol = 1e-3f,
            float lambda = 0.1f,
            IKMode mode = IKMode.LM_Chan,
            float[][] qlim = null,
            bool jointLimits = false,
            float maxAngle = 0.25f,
            System.Func<float> getRandom01 = null,
            float[] mask = null)
        {
            if (getRandom01 == null)
                getRandom01 = () => Random.value;

            // --- build mask diagonal We (6×6) ---
            float[] We = new float[6];
            if (mask == null)
            {
                for (int i = 0; i < 6; i++) We[i] = 1f;
            }
            else
            {
                for (int i = 0; i < System.Math.Min(6, mask.Length); i++)
                    We[i] = mask[i];
            }

            // --- random q0 seeds ---
            int sl = (int)slimit;
            float[][] q0Seeds = new float[sl][];
            float[] baseQ = getCurrentQ();
            q0Seeds[0] = (float[])baseQ.Clone();
            for (int s = 1; s < sl; s++)
            {
                q0Seeds[s] = new float[n];
                if (qlim != null)
                {
                    for (int i = 0; i < n; i++)
                        q0Seeds[s][i] = Mathf.Lerp(qlim[0][i], qlim[1][i], getRandom01());
                }
                else
                {
                    for (int i = 0; i < n; i++)
                        q0Seeds[s][i] = baseQ[i] + (getRandom01() - 0.5f) * 0.5f;
                }
            }

            int totalIterations = 0;
            float bestResidual = float.MaxValue;
            float[] bestQ = null;
            int bestIters = 0;
            string failReason = "iteration and search limit reached";

            for (int search = 0; search < sl; search++)
            {
                float[] q = (float[])q0Seeds[search].Clone();
                int iter = 0;

                while (iter < ilimit)
                {
                    iter++;

                    Matrix4x4 Te = fkine(q);
                    float[] e = AngleAxisError(Te, Tep);
                    float E = 0.5f * (e[0] * e[0] * We[0] + e[1] * e[1] * We[1] + e[2] * e[2] * We[2]
                                    + e[3] * e[3] * We[3] + e[4] * e[4] * We[4] + e[5] * e[5] * We[5]);

                    // convergence check
                    if (E < tol)
                    {
                        // wrap to ±π
                        for (int i = 0; i < n; i++)
                            q[i] = AngleWrap(q[i]);

                        if (jointLimits && qlim != null && !CheckJointLimits(q, qlim))
                        {
                            // solution violates joint limits, try next search
                            break;
                        }

                        return IKSolution.Ok(q, totalIterations + iter, E);
                    }

                    // --- LM step ---
                    float[][] Jfull = jacob0(q);
                    if (Jfull == null || Jfull.Length < 6) break;

                    // Build A = J^T * We * J + Wn  (n×n), b = J^T * We * e (n)
                    float[] A = new float[n * n];
                    float[] b_vec = new float[n];

                    for (int i = 0; i < n; i++)
                    {
                        // b[i] = Σ_k J[k][i] * We[k] * e[k]
                        float bi = 0f;
                        for (int k = 0; k < 6; k++)
                            bi += Jfull[k][i] * We[k] * e[k];
                        b_vec[i] = bi;

                        for (int j = 0; j < n; j++)
                        {
                            float sum = 0f;
                            for (int k = 0; k < 6; k++)
                                sum += Jfull[k][i] * We[k] * Jfull[k][j];
                            A[i * n + j] = sum;
                        }
                    }

                    // damping matrix Wn
                    float damping;
                    if (mode == IKMode.LM_Chan)
                        damping = lambda * E;          // Chan: λ·E
                    else
                        damping = lambda;               // Wampler: constant λ

                    for (int i = 0; i < n; i++)
                        A[i * n + i] += damping;

                    float[] dq = GaussianSolve(A, b_vec, n);
                    if (dq == null)
                    {
                        // singular step, abandon this search
                        break;
                    }

                    // clamp dq
                    for (int i = 0; i < n; i++)
                        dq[i] = Mathf.Clamp(dq[i], -maxAngle, maxAngle);

                    // update q
                    for (int i = 0; i < n; i++)
                        q[i] += dq[i];

                    // track best
                    if (E < bestResidual)
                    {
                        bestResidual = E;
                        bestQ = (float[])q.Clone();
                        bestIters = iter;
                    }
                }

                totalIterations += iter;
            }

            // return best found (or fallback)
            if (bestQ != null)
                return IKSolution.Ok(bestQ, totalIterations, bestResidual);

            return IKSolution.Fail(getCurrentQ(), bestResidual, failReason);
        }

        // ════════════════════════════════════════════════════════════
        //  ANGLE-AXIS ERROR
        // ════════════════════════════════════════════════════════════

        /// <summary>
        /// Compute the 6-DOF angle-axis error from current EEF pose Te to desired Tep.
        /// Returns [dx, dy, dz, ωx, ωy, ωz] where ω is the axis-angle rotation error.
        /// Equivalent to lerobot-kinematics angle_axis().
        /// </summary>
        public static float[] AngleAxisError(Matrix4x4 Te, Matrix4x4 Tep)
        {
            // Δ = Te⁻¹ * Tep
            Matrix4x4 delta = Te.inverse * Tep;

            // position error (already in Te frame)
            float px = delta.m03;
            float py = delta.m13;
            float pz = delta.m23;

            // rotation error from SO(3) part
            // R = Δ[:3,:3]
            float r00 = delta.m00, r01 = delta.m01, r02 = delta.m02;
            float r10 = delta.m10, r11 = delta.m11, r12 = delta.m12;
            float r20 = delta.m20, r21 = delta.m21, r22 = delta.m22;

            // θ = acos((trace(R) - 1) / 2)
            float traceR = r00 + r11 + r22;
            float cosTheta = (traceR - 1f) * 0.5f;
            cosTheta = Mathf.Clamp(cosTheta, -1f, 1f);
            float theta = Mathf.Acos(cosTheta);

            float wx, wy, wz;
            if (theta < 1e-6f)
            {
                // small angle approximation: ω ≈ 0.5 * [R32-R23, R13-R31, R21-R12]
                wx = 0.5f * (r21 - r12);
                wy = 0.5f * (r02 - r20);
                wz = 0.5f * (r10 - r01);
            }
            else
            {
                float coeff = theta / (2f * Mathf.Sin(theta));
                wx = coeff * (r21 - r12);
                wy = coeff * (r02 - r20);
                wz = coeff * (r10 - r01);
            }

            return new float[] { px, py, pz, wx, wy, wz };
        }

        /// <summary>
        /// Build a 4×4 homogeneous transform from position and rotation.
        /// </summary>
        public static Matrix4x4 BuildTransform(Vector3 pos, Quaternion rot)
        {
            Matrix4x4 m = Matrix4x4.TRS(pos, rot, Vector3.one);
            return m;
        }

        // ════════════════════════════════════════════════════════════
        //  DENSE JACOBIAN EXTRACTION
        // ════════════════════════════════════════════════════════════

        /// <summary>
        /// Get 6×n Jacobian from ArticulationBody as float[][] row-major.
        /// Includes all columns (including locked if any).
        /// </summary>
        public static float[][] GetDenseJacobian(ArticulationBody root, int n)
        {
            var jac = new ArticulationJacobian();
            root.GetDenseJacobian(ref jac);
            int rows = jac.rows;
            int cols = jac.columns;

            if (jac.elements == null || jac.elements.Count != rows * cols)
                return null;

            int eefStart = rows - 6;
            float[][] J = new float[6][];
            for (int i = 0; i < 6; i++)
            {
                J[i] = new float[n];
                for (int j = 0; j < System.Math.Min(n, cols); j++)
                    J[i][j] = jac.elements[(eefStart + i) * cols + j];
            }

            return J;
        }

        /// <summary>
        /// Get 6×n Jacobian with one column zeroed out (for locked joint).
        /// </summary>
        public static float[][] GetDenseJacobianWithLockedJoint(ArticulationBody root, int n, int lockedIndex)
        {
            float[][] J = GetDenseJacobian(root, n);
            if (J != null && lockedIndex >= 0 && lockedIndex < n)
            {
                for (int i = 0; i < 6; i++)
                    J[i][lockedIndex] = 0f;
            }
            return J;
        }

        // ════════════════════════════════════════════════════════════
        //  UTILITY
        // ════════════════════════════════════════════════════════════

        public static float[] GaussianSolve(float[] A, float[] b, int n)
        {
            float[] aug = new float[n * (n + 1)];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                    aug[i * (n + 1) + j] = A[i * n + j];
                aug[i * (n + 1) + n] = b[i];
            }

            for (int col = 0; col < n; col++)
            {
                int maxRow = col;
                float maxVal = Mathf.Abs(aug[col * (n + 1) + col]);
                for (int row = col + 1; row < n; row++)
                {
                    float v = Mathf.Abs(aug[row * (n + 1) + col]);
                    if (v > maxVal) { maxVal = v; maxRow = row; }
                }

                if (maxVal < 1e-10f) return null;

                if (maxRow != col)
                {
                    for (int j = 0; j <= n; j++)
                    {
                        float tmp = aug[col * (n + 1) + j];
                        aug[col * (n + 1) + j] = aug[maxRow * (n + 1) + j];
                        aug[maxRow * (n + 1) + j] = tmp;
                    }
                }

                float pivot = aug[col * (n + 1) + col];
                for (int j = col; j <= n; j++)
                    aug[col * (n + 1) + j] /= pivot;

                for (int row = 0; row < n; row++)
                {
                    if (row == col) continue;
                    float factor = aug[row * (n + 1) + col];
                    for (int j = col; j <= n; j++)
                        aug[row * (n + 1) + j] -= factor * aug[col * (n + 1) + j];
                }
            }

            float[] x = new float[n];
            for (int i = 0; i < n; i++)
                x[i] = aug[i * (n + 1) + n];
            return x;
        }

        /// <summary>
        /// Wrap angle to [-π, π].
        /// </summary>
        public static float AngleWrap(float rad)
        {
            rad = rad % (2f * Mathf.PI);
            if (rad > Mathf.PI) rad -= 2f * Mathf.PI;
            if (rad < -Mathf.PI) rad += 2f * Mathf.PI;
            return rad;
        }

        /// <summary>
        /// Check if q is within joint limits qlim[0][] <= q[i] <= qlim[1][].
        /// </summary>
        public static bool CheckJointLimits(float[] q, float[][] qlim)
        {
            if (qlim == null) return true;
            int n = System.Math.Min(q.Length, System.Math.Min(qlim[0].Length, qlim[1].Length));
            for (int i = 0; i < n; i++)
            {
                if (q[i] < qlim[0][i] || q[i] > qlim[1][i])
                    return false;
            }
            return true;
        }
    }
}