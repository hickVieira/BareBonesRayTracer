using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;

namespace RayTracer.BVH
{
    [System.Serializable]
    public struct BVHData
    {
        public Vector3 min;
        public Vector3 max;

        public int leftIndex;
        public int rightIndex;

        public int triangleIndex; // if -1 then !leaf
        public int triangleCount;

        public bool IsLeaf { get => triangleIndex != -1; }
    }

    [System.Serializable]
    public struct Triangle
    {
        public Vector3 position0;
        public Vector3 position1;
        public Vector3 position2;
        public Vector3 normal;

        public Triangle(Vector3 position0, Vector3 position1, Vector3 position2, Vector3 normal)
        {
            this.position0 = position0;
            this.position1 = position1;
            this.position2 = position2;
            this.normal = normal;
        }
    }

    [System.Serializable]
    public struct TriangleData
    {
        public int triangleIndex;
        public int nodeIndex;
        public Bounds bounds;

        public TriangleData(int triangleIndex, int nodeIndex, Bounds bounds)
        {
            this.triangleIndex = triangleIndex;
            this.nodeIndex = nodeIndex;
            this.bounds = bounds;
        }
    }

    [System.Serializable]
    public struct BVHNode
    {
        public int index;
        public Bounds bounds;
        public int leftNodeIndex;
        public int rightNodeIndex;
        public bool isLeaf;

        public BVHNode(int index, Bounds bounds, int leftNodeIndex, int rightNodeIndex, bool isLeaf)
        {
            this.index = index;
            this.bounds = bounds;
            this.leftNodeIndex = leftNodeIndex;
            this.rightNodeIndex = rightNodeIndex;
            this.isLeaf = isLeaf;
        }
    }

    [BurstCompile()]
    public struct BVHBuildTrianglesJob : IJob
    {
        [ReadOnly] public NativeArray<Vector3> inVertices;
        [ReadOnly] public NativeArray<int> inIndices;

        public int outTrianglesLength;
        [WriteOnly] public NativeList<Triangle> outTriangles;
        [WriteOnly] public NativeList<TriangleData> outTrianglesDatas;

        public Matrix4x4 localToWorldMatrix;

        public void Execute()
        {
            for (int i = 0; i < inIndices.Length; i += 3)
            {
                int vIndex0 = inIndices[i + 0];
                int vIndex1 = inIndices[i + 1];
                int vIndex2 = inIndices[i + 2];

                // triangle
                Vector3 position0 = localToWorldMatrix.MultiplyPoint(inVertices[vIndex0]);
                Vector3 position1 = localToWorldMatrix.MultiplyPoint(inVertices[vIndex1]);
                Vector3 position2 = localToWorldMatrix.MultiplyPoint(inVertices[vIndex2]);
                Vector3 normal = -Vector3.Cross(position0 - position1, position2 - position1).normalized;
                outTriangles.Add(new Triangle(position0, position1, position2, normal));

                // bounds
                Vector3 min = Vector3.Min(Vector3.Min(position0, position1), position2);
                Vector3 max = Vector3.Max(Vector3.Max(position0, position1), position2);
                outTrianglesDatas.Add(new TriangleData(outTrianglesLength, -1, new Bounds() { min = min, max = max }));

                outTrianglesLength++;
            }
        }
    }

    [BurstCompile()]
    public struct BVHNodesBuilderJob : IJob
    {
        public NativeList<BVHNode> outBVHNodes;
        public NativeArray<TriangleData> outTrianglesDatas;
        [WriteOnly] public NativeArray<int> outRootIndex;
        public int splitCount;

        public void Execute()
        {
            BVHNode rootNode = CreateBVHRecursive(outTrianglesDatas, splitCount);
            outRootIndex[0] = rootNode.index;
        }

        BVHNode CreateBVHNodeLeaf(NativeSlice<TriangleData> sliceArray)
        {
            for (int i = 0; i < sliceArray.Length; i++)
            {
                TriangleData sliceTB = sliceArray[i];
                sliceTB.nodeIndex = outBVHNodes.Length;
                sliceArray[i] = sliceTB;

                TriangleData originalTB = outTrianglesDatas[sliceTB.triangleIndex];
                originalTB.nodeIndex = outBVHNodes.Length;
                outTrianglesDatas[sliceTB.triangleIndex] = originalTB;
            }

            BVHNode newNode = new BVHNode(outBVHNodes.Length, CalcBounds(sliceArray), -1, -1, true);

            outBVHNodes.Add(newNode);
            return newNode;
        }

        BVHNode CreateBVHRecursive(NativeSlice<TriangleData> trianglesDatasArray, int splitCount, int recursiveCount = 0)
        {
            // Find smallest cost split
            // Select Axis  0 = X, 1 = Y, 2 = Z
            float bestSplit = 0f;
            int bestAxis = -1;

            if (trianglesDatasArray.Length > 3)
            {
                var (totalBounds, minCost) = CalculateBoundsAndSAH(trianglesDatasArray);
                Vector3 size = totalBounds.size;

                NativeArray<TriangleData> leftBuffer = new NativeArray<TriangleData>(trianglesDatasArray.Length, Allocator.Temp);
                NativeArray<TriangleData> rightBuffer = new NativeArray<TriangleData>(trianglesDatasArray.Length, Allocator.Temp);

                for (int axis = 0; axis < 3; ++axis)
                {
                    if (size[axis] < 0.001) continue;

                    float step = size[axis] / (splitCount / (recursiveCount + 1));

                    float stepStart = totalBounds.min[axis] + step;
                    float stepEnd = totalBounds.max[axis] - step;

                    for (float testSplit = stepStart; testSplit < stepEnd; testSplit += step)
                    {
                        var (left, right) = SplitLR(trianglesDatasArray, axis, testSplit, ref leftBuffer, ref rightBuffer);

                        if (left.Length <= 1 || right.Length <= 1) continue;

                        var (_, costLeft) = CalculateBoundsAndSAH(left);
                        var (_, costRight) = CalculateBoundsAndSAH(right);

                        float cost = costLeft + costRight;

                        if (cost < minCost)
                        {
                            minCost = cost;
                            bestAxis = axis;
                            bestSplit = testSplit;
                        }
                    }
                }

                rightBuffer.Dispose();
                leftBuffer.Dispose();
            }

            // Not Split
            if (bestAxis < 0)
            {
                return CreateBVHNodeLeaf(trianglesDatasArray);
            }
            // Calc child
            else
            {
                NativeArray<TriangleData> leftBuffer = new NativeArray<TriangleData>(trianglesDatasArray.Length, Allocator.Temp);
                NativeArray<TriangleData> rightBuffer = new NativeArray<TriangleData>(trianglesDatasArray.Length, Allocator.Temp);

                var (left, right) = SplitLR(trianglesDatasArray, bestAxis, bestSplit, ref leftBuffer, ref rightBuffer);

                BVHNode leftNode = CreateBVHRecursive(left, splitCount, recursiveCount + 1);
                BVHNode rightNode = CreateBVHRecursive(right, splitCount, recursiveCount + 1);

                Bounds bounds = leftNode.bounds;
                bounds.Encapsulate(rightNode.bounds);

                rightBuffer.Dispose();
                leftBuffer.Dispose();

                BVHNode newNode = new BVHNode(outBVHNodes.Length, bounds, leftNode.index, rightNode.index, false);
                outBVHNodes.Add(newNode);

                return newNode;
            }
        }

        Bounds CalcBounds(NativeSlice<TriangleData> trianglesDatasArray)
        {
            Vector3 min = Vector3.one * float.MaxValue;
            Vector3 max = Vector3.one * float.MinValue;

            for (var i = 0; i < trianglesDatasArray.Length; ++i)
            {
                Bounds bounds = trianglesDatasArray[i].bounds;
                min = Vector3.Min(min, bounds.min);
                max = Vector3.Max(max, bounds.max);
            }

            return new Bounds() { min = min, max = max };
        }

        // SAH(Surface Area Heuristics)
        // the current bbox has a cost of (number of triangles) * surfaceArea of C = N * SA
        (Bounds, float) CalculateBoundsAndSAH(NativeSlice<TriangleData> trianglesDatasArray)
        {
            Bounds bounds = CalcBounds(trianglesDatasArray);

            Vector3 size = bounds.size;
            float sah = trianglesDatasArray.Length * (size.x * size.y + size.x * size.y + size.y * size.z);

            return (bounds, sah);
        }

        (NativeSlice<TriangleData> left, NativeSlice<TriangleData> right) SplitLR(NativeSlice<TriangleData> triBoundsArray, int axis, float split, ref NativeArray<TriangleData> leftBuffer, ref NativeArray<TriangleData> rightBuffer)
        {
            int leftCount = 0;
            int rightCount = 0;

            for (int i = 0; i < triBoundsArray.Length; ++i)
            {
                TriangleData tb = triBoundsArray[i];

                if (tb.bounds.center[axis] < split)
                    leftBuffer[leftCount++] = tb;
                else
                    rightBuffer[rightCount++] = tb;
            }

            return (leftBuffer.Slice(0, leftCount), rightBuffer.Slice(0, rightCount));
        }
    }

    [BurstCompile()]
    public struct BVHDataBuilderJob : IJob
    {
        [ReadOnly] public NativeArray<BVHNode> inBVHNodes;
        public NativeList<Triangle> inTriangles;
        public NativeList<TriangleData> inTrianglesDatas;

        public int rootNodeIndex;
        public NativeArray<BVHData> outBVHData;
        public NativeArray<Triangle> outTriangles;

        int addedBVHDataCount;
        int addedTrianglesCount;

        public void Execute()
        {
            addedBVHDataCount = 0;
            addedTrianglesCount = 0;
            CreateBVHDataRecursive(inBVHNodes[rootNodeIndex]);
        }

        void CreateBVHDataRecursive(BVHNode node)
        {
            BVHData data = new BVHData()
            {
                min = node.bounds.min,
                max = node.bounds.max,
                leftIndex = -1,
                rightIndex = -1,
                triangleIndex = -1,
                triangleCount = 0,
            };

            if (node.isLeaf)
            {
                int beforeAddCount = addedTrianglesCount;
                AddTriangles(node.index);
                int afterAddCount = addedTrianglesCount;

                data.triangleIndex = beforeAddCount;
                data.triangleCount = afterAddCount - beforeAddCount;

                outBVHData[addedBVHDataCount++] = data;
            }
            else
            {
                data.triangleIndex = -1;

                int dataIndex = addedBVHDataCount++;

                data.leftIndex = addedBVHDataCount;
                CreateBVHDataRecursive(inBVHNodes[node.leftNodeIndex]);

                data.rightIndex = addedBVHDataCount;
                CreateBVHDataRecursive(inBVHNodes[node.rightNodeIndex]);

                outBVHData[dataIndex] = data;
            }
        }

        void AddTriangles(int nodeIndex)
        {
            for (int i = 0; i < inTrianglesDatas.Length; i++)
            {
                TriangleData td = inTrianglesDatas[i];

                if (td.nodeIndex == nodeIndex)
                {
                    outTriangles[addedTrianglesCount++] = inTriangles[td.triangleIndex];
                    inTrianglesDatas.RemoveAtSwapBack(i);
                    i--;
                }
            }
        }
    }

    public class BVHBuilder : MonoBehaviour
    {
        public static class ShaderParams
        {
            public static int bvhBuffer = Shader.PropertyToID("bvhBuffer");
            public static int bvhTrianglesBuffer = Shader.PropertyToID("bvhTrianglesBuffer");
        }
        [SerializeField] [Range(2, 128)] private int _splitCount = 64;
        [SerializeField] private Triangle[] _bvhTriangles;
        [SerializeField] private BVHData[] _bvhDatas;
        [SerializeField] private GraphicsBuffer _bvhBuffer;
        [SerializeField] private GraphicsBuffer _bvhTrianglesBuffer;

        // Debug
        public TriangleData[] trianglesDatas;
        public bool debugTriangles;
        public bool debugTrianglesDatas;
        public bool debugAllBVHNodes;
        public int debugBVHDepth;
        public bool debugOnlyLeafNode;

        void Start()
        {
            BuildBVH();
        }

        public void SetBuffers(Material material)
        {
            if (_bvhBuffer != null) _bvhBuffer.Release();
            if (_bvhTrianglesBuffer != null) _bvhTrianglesBuffer.Release();

            _bvhBuffer = new GraphicsBuffer(GraphicsBuffer.Target.Structured, _bvhDatas.Length, System.Runtime.InteropServices.Marshal.SizeOf<BVHData>());
            _bvhBuffer.SetData(_bvhDatas);
            _bvhTrianglesBuffer = new GraphicsBuffer(GraphicsBuffer.Target.Structured, _bvhTriangles.Length, System.Runtime.InteropServices.Marshal.SizeOf<Triangle>());
            _bvhTrianglesBuffer.SetData(_bvhTriangles);

            material.SetBuffer(ShaderParams.bvhBuffer, _bvhBuffer);
            material.SetBuffer(ShaderParams.bvhTrianglesBuffer, _bvhTrianglesBuffer);
        }

        void OnDrawGizmos()
        {
            if (debugTriangles || debugTrianglesDatas)
                for (int i = 0; i < trianglesDatas.Length; i++)
                {
                    TriangleData triangleBounds = trianglesDatas[i];
                    {
                        if (debugTrianglesDatas)
                        {
                            Gizmos.color = Color.green;
                            Gizmos.DrawWireCube(triangleBounds.bounds.center, triangleBounds.bounds.size);
                        }
                        if (debugTriangles)
                        {
                            if (triangleBounds.nodeIndex == -1)
                                Gizmos.color = Color.red;
                            else
                                Gizmos.color = Color.yellow;

                            Triangle triangle = _bvhTriangles[triangleBounds.triangleIndex];
                            Gizmos.DrawLine(triangle.position0, triangle.position1);
                            Gizmos.DrawLine(triangle.position0, triangle.position2);
                            Gizmos.DrawLine(triangle.position1, triangle.position2);
                        }

                    }
                }

            if (debugAllBVHNodes)
            {
                for (int i = 0; i < _bvhDatas.Length; i++)
                {
                    BVHData bvhData = _bvhDatas[i];
                    if (bvhData.IsLeaf)
                        Gizmos.color = Color.cyan;
                    else
                        Gizmos.color = Color.green;

                    Bounds bounds = new Bounds() { min = bvhData.min, max = bvhData.max };
                    Gizmos.DrawWireCube(bounds.center, bounds.size);
                }
            }
            else
            {
                DrawBVHGizmos(0, debugBVHDepth, debugOnlyLeafNode);
            }
        }

        void DrawBVHGizmos(int idx, int gizmoDepth, bool gizmoOnlyLeafNode, int recursiveCount = 0)
        {
            if (idx < 0 || _bvhDatas.Length <= idx) return;

            var data = _bvhDatas[idx];

            if (recursiveCount == gizmoDepth)
            {
                if (data.IsLeaf)
                {
                    Gizmos.color = Color.red;
                    for (var i = 0; i < data.triangleCount; ++i)
                    {
                        var tri = _bvhTriangles[i + data.triangleIndex];
                        Gizmos.DrawLine(tri.position0, tri.position1);
                        Gizmos.DrawLine(tri.position0, tri.position2);
                        Gizmos.DrawLine(tri.position1, tri.position2);
                    }
                }

                if (!gizmoOnlyLeafNode || data.IsLeaf)
                {
                    var bounds = new Bounds() { min = data.min, max = data.max };

                    Gizmos.color = data.IsLeaf ? Color.cyan : Color.green;
                    Gizmos.DrawWireCube(bounds.center, bounds.size);
                }
            }
            else if (!data.IsLeaf)
            {
                DrawBVHGizmos(data.leftIndex, gizmoDepth, gizmoOnlyLeafNode, recursiveCount + 1);
                DrawBVHGizmos(data.rightIndex, gizmoDepth, gizmoOnlyLeafNode, recursiveCount + 1);
            }
        }

        [ContextMenu("BuildBVH")]
        public void BuildBVH()
        {
            #region Build Triangles
            // buffers
            NativeList<Triangle> triangles = new NativeList<Triangle>(Allocator.TempJob);
            NativeList<TriangleData> trianglesDatas = new NativeList<TriangleData>(Allocator.TempJob);
            NativeList<BVHNode> bvhNodes = new NativeList<BVHNode>(Allocator.TempJob);

            // get meshes
            MeshRenderer[] meshRenderers = gameObject.GetComponentsInChildren<MeshRenderer>();
            for (int i = 0; i < meshRenderers.Length; i++)
            {
                MeshFilter meshFilter = meshRenderers[i].GetComponent<MeshFilter>();
                Mesh mesh = meshFilter.sharedMesh;
                if (mesh == null)
                    continue;

                NativeArray<Vector3> vertices = new NativeArray<Vector3>(mesh.vertices, Allocator.TempJob);
                for (int j = 0; j < mesh.subMeshCount; j++)
                {
                    BVHBuildTrianglesJob bvhMeshMergeJob = new BVHBuildTrianglesJob()
                    {
                        inVertices = vertices,
                        inIndices = new NativeArray<int>(mesh.GetTriangles(j), Allocator.TempJob),
                        outTrianglesLength = triangles.Length,
                        outTriangles = triangles,
                        outTrianglesDatas = trianglesDatas,
                        localToWorldMatrix = meshRenderers[i].localToWorldMatrix
                    };
                    bvhMeshMergeJob.Schedule().Complete();
                    bvhMeshMergeJob.inIndices.Dispose();
                }
                vertices.Dispose();
            }
            #endregion

            #region Build BVH Nodes
            BVHNodesBuilderJob bvhNodesBuilderJob = new BVHNodesBuilderJob()
            {
                outBVHNodes = bvhNodes,
                outTrianglesDatas = trianglesDatas,
                outRootIndex = new NativeArray<int>(1, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
                splitCount = _splitCount,
            };
            bvhNodesBuilderJob.Schedule().Complete();
            int rootNodeIndex = bvhNodesBuilderJob.outRootIndex[0];
            bvhNodesBuilderJob.outRootIndex.Dispose();
            #endregion

            // Debug
            this.trianglesDatas = trianglesDatas.ToArray();

            #region Build BVH Data
            NativeArray<BVHData> bvhDatas = new NativeArray<BVHData>(bvhNodes.Length, Allocator.TempJob);
            NativeArray<Triangle> finalTriangles = new NativeArray<Triangle>(trianglesDatas.Length, Allocator.TempJob);
            BVHDataBuilderJob bvhDataBuilderJob = new BVHDataBuilderJob()
            {
                inBVHNodes = bvhNodes,
                inTriangles = triangles,
                inTrianglesDatas = trianglesDatas,
                rootNodeIndex = rootNodeIndex,
                outBVHData = bvhDatas,
                outTriangles = finalTriangles,
            };
            bvhDataBuilderJob.Schedule().Complete();
            #endregion

            // Debug
            this._bvhTriangles = finalTriangles.ToArray();
            this._bvhDatas = bvhDatas.ToArray();

            triangles.Dispose();
            trianglesDatas.Dispose();
            bvhNodes.Dispose();
            bvhDatas.Dispose();
            finalTriangles.Dispose();
        }
    }
}