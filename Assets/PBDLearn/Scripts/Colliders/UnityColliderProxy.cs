using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace PBDLearn
{
    public class UnityColliderProxy : MonoBehaviour,ICollider
    {
        public float concatOffset = 0.05f;

        [Range(0,1)]
        public float bounciness = 1;

        protected Rigidbody _rigidbody;

        public int entityId {
            get{
                return this.GetInstanceID();
            }
        }

        void Awake(){
            _rigidbody = GetComponent<Rigidbody>();
        }

        public virtual bool GetConcatInfo(float3 fromPosition, float3 predictPosition, out ConcatInfo concatInfo)
        {
            if(this.GetClosestSurfacePoint(fromPosition, out concatInfo)){
                concatInfo.position += concatInfo.normal * concatOffset;
                return true;
            }else if(this.GetClosestSurfacePoint(predictPosition,out concatInfo)){
                concatInfo.position += concatInfo.normal * concatOffset;
                return true;
            }
            concatInfo = default;
            return false;
        }

        public Rigidbody attachedRigidbody{
            get{
                return _rigidbody;
            }
        }

        public virtual bool PointInside(float3 p){
            return false;
        }

        public virtual bool GetClosestSurfacePoint(float3 p,out ConcatInfo closestPoint){
            closestPoint = default(ConcatInfo);
            return false;
        }

        public virtual void FillColliderDesc(CollidersGroup collidersGroup)
        {
        }
    }
}
