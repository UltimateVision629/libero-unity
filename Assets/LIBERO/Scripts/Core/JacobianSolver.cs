using UnityEngine;

namespace LIBERO.Core
{
    public static class JacobianSolver
    {
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

        private static float[] GaussianSolve(float[] A, float[] b, int n)
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
    }
}