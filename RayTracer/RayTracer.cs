using UnityEngine;

namespace RayTracer
{
    public class RayTracer : MonoBehaviour
    {
        public Material ptMaterial;
        [SerializeField] private BVH.BVHBuilder _bvhBuilder;

        void Start()
        {
            BakeBuffers();
        }

        [ContextMenu("BakeBuffers")]
        public void BakeBuffers()
        {
            if (ptMaterial != null)
            {
                _bvhBuilder = FindObjectOfType<BVH.BVHBuilder>();
                _bvhBuilder.BuildBVH();
                _bvhBuilder.SetBuffers(ptMaterial);
            }
        }
    }
}
