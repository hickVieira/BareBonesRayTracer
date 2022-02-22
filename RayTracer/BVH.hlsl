#ifndef BVH_INCLUCDE
    #define BVH_INCLUCDE
    
    #define BVH_STACK_SIZE 64
    #define BVH_FLOAT_MAX 3.402823466e+38f

    struct BVHData 
    {
        float3 min;
        float3 max;

        int leftIndex;
        int rightIndex;

        int triangleIndex; // -1 if data is not leaf
        int triangleCount;
    };

    struct BVHTriangle 
    {
        float3 position0;
        float3 position1;
        float3 position2;
        float3 normal;
    };

    StructuredBuffer<BVHData> bvhBuffer;
    StructuredBuffer<BVHTriangle> bvhTrianglesBuffer;

    inline float determinant(float3 v0, float3 v1, float3 v2)
    {
        return determinant(float3x3(
        v0.x, v1.x, v2.x,
        v0.y, v1.y, v2.y,
        v0.z, v1.z, v2.z
        ));
    }

    // Line triangle
    // https://shikousakugo.wordpress.com/2012/06/27/ray-intersection-2/
    inline bool LineTriangleIntersection(BVHTriangle tri, float3 origin, float3 rayStep, out float rayScale)
    {
        rayScale = BVH_FLOAT_MAX;

        float3 normal = tri.normal;
        float dirDot = dot(normal, rayStep);
        if (dirDot > 0) return false;

        float3 origin_from_pos0 = origin - tri.position0;
        if(dot(origin_from_pos0, normal) < 0) return false;

        float3 rayStep_end_from_pos0 = origin_from_pos0 + rayStep;
        if(dot(rayStep_end_from_pos0, normal) > 0) return false;

        float3 edge0 = tri.position1 - tri.position0;
        float3 edge1 = tri.position2 - tri.position0;

        const float float_epsilon = 0.0001;

        float d = determinant(edge0, edge1, -rayStep);
        // if (d > float_epsilon)
        {
            float dInv = 1.0 / d;
            float u = determinant(origin_from_pos0, edge1, -rayStep) * dInv;
            float v = determinant(edge0, origin_from_pos0, -rayStep) * dInv;

            if (0<=u && u<=1 && 0<=v && (u+v)<=1)
            {
                float t = determinant(edge0, edge1, origin_from_pos0) * dInv;
                if (t > 0)
                {
                    rayScale = t;
                    return true;
                }
            }
        }

        return false;
    }

    bool LineTriangleIntersectionAll(float3 origin, float3 rayStep, out float rayScale, out float3 normal)
    {
        uint num, stride;
        bvhTrianglesBuffer.GetDimensions(num, stride);

        rayScale = BVH_FLOAT_MAX;
        for(uint i=0; i<num; ++i)
        {
            BVHTriangle tri = bvhTrianglesBuffer[i];

            float tmpRayScale;
            if (LineTriangleIntersection(tri, origin, rayStep, tmpRayScale))
            {
                if (tmpRayScale < rayScale)
                {
                    rayScale = tmpRayScale;
                    normal = tri.normal;
                }
            }
        }

        return rayScale != BVH_FLOAT_MAX;
    }

    // Line AABB
    // http://marupeke296.com/COL_3D_No18_LineAndAABB.html
    bool LineAABBIntersection(float3 origin, float3 rayStep, BVHData data)
    {
        float3 aabbMin = data.min;
        float3 aabbMax = data.max;

        float tNear = -BVH_FLOAT_MAX;
        float tFar  =  BVH_FLOAT_MAX;

        for(int axis = 0; axis<3; ++axis)
        {
            float rayOnAxis = rayStep[axis];
            float originOnAxis = origin[axis];
            float minOnAxis = aabbMin[axis];
            float maxOnAxis = aabbMax[axis];
            if(rayOnAxis == 0)
            {
                if (originOnAxis < minOnAxis || maxOnAxis < originOnAxis) return false;
            }
            else
            {
                float rayOnAxisInv = 1.0 / rayOnAxis;
                float t0 = (minOnAxis - originOnAxis) * rayOnAxisInv;
                float t1 = (maxOnAxis - originOnAxis) * rayOnAxisInv;

                float tMin = min(t0, t1);
                float tMax = max(t0, t1);

                tNear = max(tNear, tMin);
                tFar  = min(tFar, tMax);

                if (tFar < 0.0 || tFar < tNear || 1.0 < tNear) return false;
            }
        }

        return true;
    }

    // Line Bvh
    // http://raytracey.blogspot.com/2016/01/gpu-path-tracing-tutorial-3-take-your.html
    bool TraverseBVH(float3 origin, float3 rayStep, out float rayScale, out float3 normal, out float heat)
    {
        int stack[BVH_STACK_SIZE];

        int stackIdx = 0;
        stack[stackIdx++] = 0;

        rayScale = BVH_FLOAT_MAX;

        while(stackIdx)
        {
            stackIdx--;
            int BvhIdx = stack[stackIdx];
            BVHData data = bvhBuffer[BvhIdx];

            if (LineAABBIntersection(origin, rayStep, data))
            {
                // Branch node
                if (data.triangleIndex < 0)
                {
                    if (stackIdx+1 >= BVH_STACK_SIZE) return false;

                    stack[stackIdx++] = data.leftIndex;
                    stack[stackIdx++] = data.rightIndex;
                }
                // Leaf node
                else
                {
                    UNITY_LOOP
                    for(int i = 0; i < data.triangleCount; ++i)
                    {
                        BVHTriangle tri = bvhTrianglesBuffer[i + data.triangleIndex];

                        float tmpRayScale;
                        if (LineTriangleIntersection(tri, origin, rayStep, tmpRayScale))
                        {
                            if (tmpRayScale < rayScale)
                            {
                                rayScale = tmpRayScale;
                                normal = tri.normal;
                            }
                        }
                        heat++;
                    }
                }
            }
        }

        return rayScale != BVH_FLOAT_MAX;
    }

#endif // BVH_INCLUCDE