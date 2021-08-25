using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace PBDLearn
{

    public interface IColliderDesc{

    }
    
    /// <summary>
    /// 球体
    /// </summary>
    public struct SphereDesc:IColliderDesc{

        public float3 center;
        public float radius;
    }


    /// <summary>
    /// 平行六面体
    /// </summary>
    public struct BoxDesc:IColliderDesc{
        public float3 min;

        //3个边轴,xyz为normalzied的朝向，w为长度
        public float4 ax; 
        public float4 ay;
        public float4 az;
    }


    /// <summary>
    /// 胶囊体
    /// </summary>
    public struct CapsuleDesc:IColliderDesc{
        public float3 c0;
        public float4 axis; //xyz为单位化的方向，w为长度
        public float radius;
    }


    public struct RigidbodyDesc{
        public float mass;
        public float bounciness;
        public float3 velocity;
    }


}
