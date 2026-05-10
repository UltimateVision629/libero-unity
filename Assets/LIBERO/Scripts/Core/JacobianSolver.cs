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
            Debug.Log($"[IK] Jacobian dims: rows={rows} cols={cols} elements={jac.elements?.Count ?? 0}");

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

            Debug.Log($"[IK] dTheta[0..2]={dTheta[0]:F5}, {dTheta[1]:F5}, {dTheta[2]:F5}");
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
