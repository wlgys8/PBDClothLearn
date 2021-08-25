using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace PBDLearn
{
    public class UnityCapsuleColliderProxy : UnityColliderProxy
    {
    
        private CapsuleDesc _desc;
        private CapsuleCollider _collider;
        private int _lastUpdateFrame;

        private CapsuleCollider unityCollider{
            get{
                if(!_collider){
                    _collider = GetComponent<CapsuleCollider>();
                }
                return _collider;
            }
        }

        public CapsuleDesc desc{
            get{
                if(_lastUpdateFrame == Time.frameCount){
                    return _desc;
                }
                _lastUpdateFrame = Time.frameCount;
                var collider = this.unityCollider;
                var localToWorld = collider.transform.localToWorldMatrix;
                var c = collider.center;
                var axis = collider.transform.up;
                var localScale = collider.transform.localScale;
                float axisScale = localScale.y;
                float radiusScale = localScale.x;
                if(collider.direction == 0){
                    axis = collider.transform.right;
                    axisScale = localScale.x;
                    radiusScale = localScale.y;
                }else if(collider.direction == 2){
                    axis = collider.transform.forward;
                    axisScale = localScale.z;
                    radiusScale = localScale.x;
                }
                c[collider.direction] = c[collider.direction] - collider.height * 0.5f + collider.radius;
                _desc.c0 = localToWorld.MultiplyPoint3x4(c);
                _desc.radius = collider.radius * radiusScale;
                _desc.axis = new float4(axis,(collider.height  - 2 * collider.radius) * axisScale);
                return _desc;
            }
        }

        public override bool PointInside(float3 p)
        {
            return IntersectUtil.PointInside(p,desc);
        }

        public override bool GetClosestSurfacePoint(float3 p,out ConcatInfo concatInfo){
            return IntersectUtil.GetClosestSurfacePoint(p,desc,out concatInfo);
        }

        public override void FillColliderDesc(CollidersGroup collidersGroup)
        {
            float mass = 0;
            if(_rigidbody && !_rigidbody.isKinematic){
                mass = _rigidbody.mass;
            }
            collidersGroup.AddCapsule(this.desc,new RigidbodyDesc(){
                mass = mass,
                bounciness = bounciness
            },this.entityId);
        }

    }
}
